#include "consensus.hpp"

Consensus::Consensus() {}

Consensus::~Consensus() {
  
}

void Consensus::Init(float _lower_L_bound, float _local_offset, float _local_gains[3], float _local_cost) {
  lower_L_bound = _lower_L_bound;
  local_offset = _local_offset;
  local_cost = _local_cost;
  for(byte i = 0; i < 3; i++) {
    local_gains[i] = (_local_gains[i] * 255.0) / 100.0;
  }
}

//Executes the needed subproblems (the global minimum, or all 6 if needed) and returns the proposed dimming vector for this node to be sent to other nodes
void Consensus::computeValueToSend(byte my_address, int number_of_addresses, byte nodes_addresses[4]) {
  float *proposedDimmingVector = computeGlobalMinimum( my_address, number_of_addresses-1, nodes_addresses);

  if(!FeasibilityCheck(proposedDimmingVector, my_address, number_of_addresses-1)) {
    proposedDimmingVector = computeBoundarySolutions(my_address, number_of_addresses-1, nodes_addresses);
  }

  Serial.print("Values to send ");
  for(byte i=0; i < number_of_addresses-1; i++) {
    Serial.print(String(proposedDimmingVector[i]) + " ");
    dimmings[retrieve_index(nodes_addresses, number_of_addresses, my_address) - 1][i] = proposedDimmingVector[i];
  }
  
}

/*
* Check if generated solutions is within node constraints
*/
bool Consensus::FeasibilityCheck(float dimming_to_check[3], byte my_address, int number_of_nodes) {
  float total_lux = 0;

  for(byte i = 0; i < number_of_nodes; i++) {
    if(dimming_to_check[i] < lower_actuator_bound) {
      return false;
    }
    if(dimming_to_check[i] > upper_actuator_bound) {
      return false;
    }
    total_lux += local_gains[i] * dimming_to_check[i];
  }
  
  if( total_lux < (lower_L_bound - local_offset) ) {
    Serial.println("Total Lux " + String(total_lux));
    return false;
  }
  return true;
}

float *Consensus::computeGlobalMinimum(byte my_address, int number_of_nodes, byte nodes_addresses[4]) {
  float *proposedDimmingVector = (float*)calloc(number_of_nodes, sizeof(float));
  for(byte i = 0; i < number_of_nodes; i++) {
    if(i == (retrieve_index(nodes_addresses, number_of_nodes+1, my_address) - 1) ) {
      proposedDimmingVector[i] = avg_dimming[i] - ( (1/optimization_rho) * ( lagrange_multipliers[i] + local_cost ) );
    } else {
      proposedDimmingVector[i] = avg_dimming[i] - ( (1/optimization_rho) * lagrange_multipliers[i] );
    }
  }
  return proposedDimmingVector;
}

float *Consensus::computeBoundarySolutions(byte my_address, int number_of_nodes, byte nodes_addresses[4]) {
  float dimming_vector_ILB[3] = {0};
  float dimming_vector_DLB[3] = {0};
  float dimming_vector_DUB[3] = {0};
  float dimming_vector_ILBintersectDLB[3] = {0};
  float dimming_vector_ILBintersectDUB[3] = {0};

  float gains_times_zed = 0;
  float zed[3] = {0};
  int my_index = retrieve_index(nodes_addresses, number_of_nodes+1, my_address) - 1;
  
  for(byte i=0; i<number_of_nodes; i++ ) {
    if(i == my_index) {
      zed[i] = optimization_rho * avg_dimming[my_index] - local_cost - lagrange_multipliers[my_index];
    } else {
      zed[i] = optimization_rho * avg_dimming[i] - lagrange_multipliers[i];
    }
    gains_times_zed += zed[i]*local_gains[i];
  }
  float my_zed = optimization_rho * avg_dimming[my_index] - local_cost - lagrange_multipliers[my_index];
  for(byte i = 0; i < number_of_nodes; i++) {
    float norm_in_fraction = ( local_gains[i] / ( pow(computeNorm(local_gains, number_of_nodes), 2) - pow(local_gains[my_index], 2) ));
    //ILB:
    dimming_vector_ILB[i] = (1/optimization_rho)*zed[i] - (local_gains[i] / pow(computeNorm(local_gains, number_of_nodes), 2))*
      (local_offset - lower_L_bound + (1/optimization_rho)*gains_times_zed);
    if(i == my_index) {
      dimming_vector_DLB[i] = 0;
      dimming_vector_DUB[i] = 100;
      dimming_vector_ILBintersectDLB[i] = 0;
      dimming_vector_ILBintersectDUB[i] = 100;
    } else {
      dimming_vector_DLB[i] = (1/optimization_rho) * zed[i];
      dimming_vector_DUB[i] = (1/optimization_rho) * zed[i];
      dimming_vector_ILBintersectDLB[i] = (1/optimization_rho)*zed[i] - 
        ( norm_in_fraction * ( local_offset - lower_L_bound - (1/optimization_rho)*( local_gains[my_index]*my_zed - gains_times_zed ) ) );

      dimming_vector_ILBintersectDUB[i] = (1/optimization_rho) * zed[i] - ( norm_in_fraction * ( local_offset - lower_L_bound + 100*local_gains[my_index] + (1/optimization_rho)*(local_gains[my_index]*my_zed - gains_times_zed) ) );
    }
  }

    //Now only the accepted solutions remain
  float bestSolution = 10000; //Solutions can't be bigger than 100% of dimming, so 101 is a nice "infinity" value
  float *bestVector;

  if( FeasibilityCheck(dimming_vector_ILB, my_address, number_of_nodes) and (computeCost(dimming_vector_ILB, number_of_nodes, my_index) < bestSolution) ) {
    bestSolution = computeCost(dimming_vector_ILB, number_of_nodes, my_index);
    bestVector = dimming_vector_ILB;
  }
  if( FeasibilityCheck(dimming_vector_DLB, my_address, number_of_nodes) and (computeCost(dimming_vector_DLB, number_of_nodes, my_index) < bestSolution) ) {
    bestSolution = computeCost(dimming_vector_DLB, number_of_nodes, my_index);
    bestVector = dimming_vector_DLB;
  }
  if( FeasibilityCheck(dimming_vector_DUB, my_address, number_of_nodes) and (computeCost(dimming_vector_DUB, number_of_nodes, my_index) < bestSolution) ) {
    bestSolution = computeCost(dimming_vector_DUB, number_of_nodes, my_index);
    bestVector = dimming_vector_DUB;
  }
  if( FeasibilityCheck(dimming_vector_ILBintersectDLB, my_address, number_of_nodes) and (computeCost(dimming_vector_ILBintersectDLB, number_of_nodes, my_index) < bestSolution) ) {
    bestSolution = computeCost(dimming_vector_ILBintersectDLB, number_of_nodes, my_index);
    bestVector = dimming_vector_ILBintersectDLB;
  }
  if( FeasibilityCheck(dimming_vector_ILBintersectDUB, my_address, number_of_nodes) and (computeCost(dimming_vector_ILBintersectDUB, number_of_nodes, my_index) < bestSolution) ) {
    bestSolution = computeCost(dimming_vector_ILBintersectDUB, number_of_nodes, my_index);
    bestVector = dimming_vector_ILBintersectDUB;
  }
  //TODO: Check if we need to watchout for a null solution ( none of the 5 was feasible)
  if(bestSolution == 10000) {
    bestVector = dimmings[my_index];
  }
  return bestVector;
}

float Consensus::computeCost(float vector_dimming[3], int number_of_nodes, byte my_index) {
  float cost = 0;
  float vector_for_norm[3] = {0};
  for(byte i=0; i<number_of_nodes; i++) {
    vector_for_norm[i] = vector_dimming[i] - avg_dimming[i];
  }
  for(byte i=0; i<number_of_nodes; i++) {
    if(i == my_index) {
      cost += local_cost*vector_dimming[i] + lagrange_multipliers[i]*(vector_dimming[i]-avg_dimming[i]);
    }else{
      cost += lagrange_multipliers[i]*(vector_dimming[i]-avg_dimming[i]);
    }
  }
  cost += 0.5*optimization_rho*( pow(computeNorm(vector_for_norm, number_of_nodes), 2) );
  Serial.println("COST " + String(cost));
  return cost;
}


//************ UPDATE LAGRANGE MULTIPLIERS *******
void Consensus::updateLagrandeMultipliers(byte my_address, int number_of_addresses, byte nodes_addresses[4]) {
  float *proposed_dimmings_vals = dimmings[(retrieve_index(nodes_addresses, number_of_addresses, my_address) - 1)];
  Serial.print("Lagrange: ");
  for(byte i = 0; i < number_of_addresses-1; i++) {
    lagrange_multipliers[i] += optimization_rho * ( proposed_dimmings_vals[i] - avg_dimming[i] );
    Serial.print(String(lagrange_multipliers[i]) + " ");
  }
}

//******** UPDATE AVERAGE VECTOR ************
void Consensus::updateAverage(int number_of_addresses) {
  for(byte i=0; i<number_of_addresses-1; i++) {
    avg_dimming[i] = 0;
    for(byte j=0; j<number_of_addresses-1; j++){
      avg_dimming[i] += dimmings[j][i];
    }
    avg_dimming[i] = avg_dimming[i] / ((float) (number_of_addresses-1));
  }
}

//*********** GETTERS AND SETTERS *************
float *Consensus::getDimmings( byte my_address, int number_of_addresses, byte nodes_addresses[4]) {
  return dimmings[(retrieve_index(nodes_addresses, number_of_addresses, my_address) - 1)];
}

int Consensus::getCurrentIteration() {
  return current_num_of_iterations;
}

float Consensus::getMyDimming(byte my_address, int number_of_addresses, byte nodes_addresses[4]) {
  return dimmings[(retrieve_index(nodes_addresses, number_of_addresses, my_address) - 1)][(retrieve_index(nodes_addresses, number_of_addresses, my_address) - 1)];
}

void Consensus::updateDimmings( byte my_address, int number_of_addresses, byte nodes_addresses[4], float tmpDimmings[3][3] ) {
  for(byte i=0; i<number_of_addresses-1; i++) {
    if(i != (retrieve_index(nodes_addresses, number_of_addresses, my_address) - 1)) {
      for(byte j=0; j<number_of_addresses-1; j++) {
        dimmings[i][j] = tmpDimmings[i][j];
      }
    }
  }
}

void Consensus::incrementIterations() {
  current_num_of_iterations++;
}
