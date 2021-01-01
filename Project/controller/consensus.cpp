#include "consensus.hpp"

Consensus::Consensus() {}

Consensus::~Consensus() {
  
}

void Consensus::Init(float _lower_L_bound, float _local_offset, float _local_gains[3], float _local_cost) {
  lower_L_bound = _lower_L_bound;
  local_offset = _local_offset;
  local_cost = _local_cost;
  for(int i = 0; i < 3; i++) {
    local_gains[i] = (_local_gains[i] * 100) / 255.0;
  }
}

//Executes the needed subproblems (the global minimum, or all 6 if needed) and returns the proposed dimming vector for this node to be sent to other nodes
void Consensus::computeValueToSend(float avg_dimming[3], byte my_address, int number_of_addresses, byte nodes_addresses[4]) {
  float *proposedDimmingVector = computeGlobalMinimum(avg_dimming, my_address, number_of_addresses-1, nodes_addresses);
  
  if(!FeasibilityCheck(proposedDimmingVector, my_address, number_of_addresses-1)) {
    proposedDimmingVector = computeBoundarySolutions(avg_dimming, my_address, number_of_addresses-1, nodes_addresses);
  }
  
  for(int i=0; i < number_of_addresses-1; i++) {
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
    return false;
  }
  return true;
}

float *Consensus::computeGlobalMinimum(float avg_dimming[3], byte my_address, int number_of_nodes, byte nodes_addresses[4]) {
  float proposedDimmingVector[3];
  for(byte i = 0; i < number_of_nodes; i++) {
    if(i == (retrieve_index(nodes_addresses, number_of_nodes+1, my_address) - 1) ) {
      proposedDimmingVector[i] = avg_dimming[i] - ( (1/optimization_rho) * ( lagrange_multipliers[i] + local_cost ) );
    } else {
      proposedDimmingVector[i] = avg_dimming[i] - ( (1/optimization_rho) * lagrange_multipliers[i] );
    }
  }
  return proposedDimmingVector;
}

float *Consensus::computeBoundarySolutions(float avg_dimming[3], byte my_address, int number_of_nodes, byte nodes_addresses[4]) {
  float dimming_vector_ILB[3] = {0};
  float dimming_vector_DLB[3] = {0};
  float dimming_vector_DUB[3] = {0};
  float dimming_vector_ILBintersectDLB[3] = {0};
  float dimming_vector_ILBintersectDUB[3] = {0};

  int my_index = retrieve_index(nodes_addresses, number_of_nodes+1, my_address) - 1;
  float my_zed = optimization_rho * avg_dimming[my_index] - local_cost - lagrange_multipliers[my_index];
  for(byte i = 0; i < number_of_nodes; i++) {
    float zed = optimization_rho * avg_dimming[i] - local_cost - lagrange_multipliers[i];
    float norm_in_fraction = ( local_gains[i] / ( pow(computeNorm(local_gains, number_of_nodes), 2) - pow(local_gains[my_index], 2) ));
    //ILB:
    dimming_vector_ILB[i] = (1/optimization_rho)*zed - (local_gains[i] / pow(computeNorm(local_gains, number_of_nodes), 2))*
      (local_offset - lower_L_bound + (1/optimization_rho)*local_gains[i]*zed);
    if(i == (retrieve_index(nodes_addresses, number_of_nodes+1, my_address) - 1)) {
      dimming_vector_DLB[i] = 0;
      dimming_vector_DUB[i] = 100;
      dimming_vector_ILBintersectDLB[i] = 0;
      dimming_vector_ILBintersectDUB[i] = 100;
    } else {
      dimming_vector_DLB[i] = (1/optimization_rho) * zed;
      dimming_vector_DUB[i] = (1/optimization_rho) * zed;
      dimming_vector_ILBintersectDLB[i] = (1/optimization_rho)*zed - 
        ( norm_in_fraction * ( local_offset - lower_L_bound - (1/optimization_rho)*( local_gains[my_index]*my_zed - local_gains[i]*zed ) ) );

      dimming_vector_ILBintersectDUB[i] = (1/optimization_rho) * zed - ( norm_in_fraction * ( local_offset - lower_L_bound + 100*local_gains[my_index] + (1/optimization_rho)*(local_gains[my_index]*my_zed - local_gains[i]*zed) ) );
    }
  }

    //Now only the accepted solutions remain
  float bestSolution = 101; //Solutions can't be bigger than 100% of dimming, so 101 is a nice "infinity" value
  float *bestVector;

  if( FeasibilityCheck(dimming_vector_ILB, my_address, number_of_nodes) and (computeNorm(dimming_vector_ILB, number_of_nodes) < bestSolution) ) {
    bestSolution = computeNorm(dimming_vector_ILB, number_of_nodes);
    bestVector = dimming_vector_ILB;
  }
  if( FeasibilityCheck(dimming_vector_DLB, my_address, number_of_nodes) and (computeNorm(dimming_vector_DLB, number_of_nodes) < bestSolution) ) {
    bestSolution = computeNorm(dimming_vector_DLB, number_of_nodes);
    bestVector = dimming_vector_DLB;
  }
  if( FeasibilityCheck(dimming_vector_DUB, my_address, number_of_nodes) and (computeNorm(dimming_vector_DUB, number_of_nodes) < bestSolution) ) {
    bestSolution = computeNorm(dimming_vector_DUB, number_of_nodes);
    bestVector = dimming_vector_DUB;
  }
  if( FeasibilityCheck(dimming_vector_ILBintersectDLB, my_address, number_of_nodes) and (computeNorm(dimming_vector_ILBintersectDLB, number_of_nodes) < bestSolution) ) {
    bestSolution = computeNorm(dimming_vector_ILBintersectDLB, number_of_nodes);
    bestVector = dimming_vector_ILBintersectDLB;
  }
  if( FeasibilityCheck(dimming_vector_ILBintersectDUB, my_address, number_of_nodes) and (computeNorm(dimming_vector_ILBintersectDUB, number_of_nodes) < bestSolution) ) {
    bestSolution = computeNorm(dimming_vector_ILBintersectDUB, number_of_nodes);
    bestVector = dimming_vector_ILBintersectDUB;
  }
  //TODO: Check if we need to watchout for a null solution ( none of the 5 was feasible)
  
  return bestVector;
}


//************ UPDATE LAGRANGE MULTIPLIERS *******
void Consensus::updateLagrandeMultipliers(float avg_dimming[3], float proposed_dimming_vals[3], int number_of_nodes) {
  for(byte i = 0; i < number_of_nodes; i++) {
    lagrange_multipliers[i] += optimization_rho * ( proposed_dimming_vals[i] - avg_dimming[i] );
  }
}
