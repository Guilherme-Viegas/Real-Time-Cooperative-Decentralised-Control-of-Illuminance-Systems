#include "consensus.hpp"

Consensus::Consensus() {}

Consensus::~Consensus() {
  free(local_gains_vector);
  free(lagrange_multipliers);
  free(proposed_dimming_vals);
}

void Consensus::Init(float _lower_L_bound, float _local_offset, float _local_cost, float *_local_gains_vector, int _number_of_nodes, byte _my_addr_vector_index) {
  current_num_of_iterations = 0;
  lower_L_bound = _lower_L_bound;
  local_offset = _local_offset;
  local_cost = _local_cost;
  number_of_nodes = _number_of_nodes;
  my_addr_vector_index = _my_addr_vector_index;

  local_gains_vector = (float*)malloc(_number_of_nodes*sizeof(float));
  for(byte i = 0; i<_number_of_nodes; i++) {
    local_gains_vector[i] = (_local_gains_vector[i] * 100) / 255.0;
  }
  lagrange_multipliers = (float*)calloc(_number_of_nodes, sizeof(float));
  proposed_dimming_vals = (float*)calloc(_number_of_nodes, sizeof(float));
}

//Executes the needed subproblems (the global minimum, or all 6 if needed) and returns the proposed dimming for this node to be sent to other nodes
float Consensus::computeValueToSend(float avg_dimming) {
  float proposed_dimming;
  if( (proposed_dimming = computeGlobalMinimum(avg_dimming)) != -1) {
    return proposed_dimming;
  } else {
    return computeBoundarySolutions(avg_dimming); 
  }
}

/*
* Check if generated solutions is within node constraints
*/
bool Consensus::FeasibilityCheck(float *dimming_to_check) {
  float total_lux = 0;
  if(dimming_to_check[my_addr_vector_index] < lower_actuator_bound) {
    return false;
  }
  if(dimming_to_check[my_addr_vector_index] > upper_actuator_bound) {
    return false;
  }

  for(byte i = 0; i < number_of_nodes; i++) {
    total_lux += local_gains_vector[i] * dimming_to_check[i];
  }
  if( total_lux < (lower_L_bound - local_offset) ) {
    return false;
  }

  return true;
}

//********** COST FUNCTION SOLUTIONS **************

float Consensus::computeGlobalMinimum(float avg_dimming) {
  float *proposedDimmingVector = (float*)malloc(number_of_nodes * sizeof(float));
  float proposedDimming = -1;

  for(byte i = 0; i < number_of_nodes; i++) {
    if(i == my_addr_vector_index) {
      proposedDimmingVector[i] = avg_dimming - ( (1/optimization_rho) * ( lagrange_multipliers[i] + local_cost ) );
    } else {
      proposedDimmingVector[i] = avg_dimming - ( (1/optimization_rho) * lagrange_multipliers[i] );
    }
  }

  proposedDimming = proposedDimmingVector[my_addr_vector_index];
  if( FeasibilityCheck(proposedDimmingVector) ) { //If it passes Feasibility, because it's the global minimum it's the best solution, so we store the proposed solution vector
    for(byte i = 0; i < number_of_nodes; i++) {
      proposed_dimming_vals[i] = proposedDimmingVector[i];
    }
    return proposedDimming;
  } else {
    free(proposedDimmingVector);
    return -1;
  }
}

float Consensus::computeBoundarySolutions(float avg_dimming) {
  float proposedDimming = -1;
  float *dimming_vector_ILB = (float*)malloc(number_of_nodes * sizeof(float));
  float *dimming_vector_DLB = (float*)malloc(number_of_nodes * sizeof(float));
  float *dimming_vector_DUB = (float*)malloc(number_of_nodes * sizeof(float));
  float *dimming_vector_ILBintersectDLB = (float*)malloc(number_of_nodes * sizeof(float));
  float *dimming_vector_ILBintersectDUB = (float*)malloc(number_of_nodes * sizeof(float));

  for(byte i = 0; i < number_of_nodes; i++) {
    if(i == my_addr_vector_index) {
      //Compute d_ii
    } else {
      //Compute d_ij
    }
  }

  if(!FeasibilityCheck(dimming_vector_ILB)) {
    free(dimming_vector_ILB);
  }
  if(!FeasibilityCheck(dimming_vector_DLB)) {
    free(dimming_vector_DLB);
  }
  if(!FeasibilityCheck(dimming_vector_DUB)) {
    free(dimming_vector_DUB);
  }
  if(!FeasibilityCheck(dimming_vector_ILBintersectDLB)) {
    free(dimming_vector_ILBintersectDLB);
  }
  if(!FeasibilityCheck(dimming_vector_ILBintersectDUB)) {
    free(dimming_vector_ILBintersectDUB);
  }
  //Now only the accepted solutions remain
  float bestSolution = 101; //Solutions can't be bigger than 100% of dimming, so 101 is a nice "infinity" value
  float *bestVector;

  //Save the best one
  if( (dimming_vector_ILB != NULL) && (dimming_vector_ILB[my_addr_vector_index] < bestSolution) ) {
    bestSolution = dimming_vector_ILB[my_addr_vector_index];
    bestVector = dimming_vector_ILB;
  } else {
    free(dimming_vector_ILB);
  }
  if( (dimming_vector_DLB != NULL) && (dimming_vector_DLB[my_addr_vector_index] < bestSolution) ) {
    bestSolution = dimming_vector_DLB[my_addr_vector_index];
    bestVector = dimming_vector_DLB;
  } else {
    free(dimming_vector_DLB);
  }
  if( (dimming_vector_DUB != NULL) && (dimming_vector_DUB[my_addr_vector_index] < bestSolution) ) {
    bestSolution = dimming_vector_DUB[my_addr_vector_index];
    bestVector = dimming_vector_DUB;
  } else {
    free(dimming_vector_DUB);
  }
  if( (dimming_vector_ILBintersectDLB != NULL) && (dimming_vector_ILBintersectDLB[my_addr_vector_index] < bestSolution) ) {
    bestSolution = dimming_vector_ILBintersectDLB[my_addr_vector_index];
    bestVector = dimming_vector_ILBintersectDLB;
  } else {
    free(dimming_vector_ILBintersectDLB);
  }
  if( (dimming_vector_ILBintersectDUB != NULL) && (dimming_vector_ILBintersectDUB[my_addr_vector_index] < bestSolution) ) {
    bestSolution = dimming_vector_ILBintersectDUB[my_addr_vector_index];
    bestVector = dimming_vector_ILBintersectDUB;
  } else {
    free(dimming_vector_ILBintersectDUB);
  }

  if(bestSolution == 101) {
    return -1;
  }
  else {
    for(byte i = 0; i < number_of_nodes; i++) {
      proposed_dimming_vals[i] = bestVector[i];
    }
    return bestSolution;
  }
}

//************ UPDATE LAGRANGE MULTIPLIERS *******
void Consensus::updateLagrandeMultipliers(float avg_dimming) {
  for(byte i = 0; i < number_of_nodes; i++) {
    lagrange_multipliers[i] += optimization_rho * ( proposed_dimming_vals[i] - avg_dimming );
  }
}
