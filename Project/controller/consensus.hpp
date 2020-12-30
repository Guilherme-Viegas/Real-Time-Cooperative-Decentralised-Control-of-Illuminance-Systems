#ifndef CONSENSUS_HPP
#define CONSENSUS_HPP

#include <Math.h>
#include "Arduino.h"

#define optimization_rho 0.07 //Value teacher used, but we may need to adjust
#define max_iterations 20

class Consensus {
    int current_num_of_iterations = 0;
    float lower_L_bound = -1;
    float local_offset = -1;
    float local_cost = -1;
    float lower_actuator_bound = 0; // Lower and Upper dimming values [0, 100] in our case
    float upper_actuator_bound = 100;
    float *local_gains_vector; //Vector of gains with respect to dimming values [0, 100]
    int number_of_nodes = 0;
    byte my_addr_vector_index = 0; // If I'm node 4 and addr_vector is [0, 1, 2, 4, 7] then my_addr_vector_index == 2, broadcast does not count

    float *lagrange_multipliers;
    float *proposed_dimming_vals;

  public:
    Consensus();
    ~Consensus();
    void Init(float _lower_L_bound, float _local_offset, float _local_cost, float *_local_gains_vector, int _number_of_nodes, byte _my_addr_vector_index); //Constructor
    float computeValueToSend(float avg_dimming);
    bool FeasibilityCheck(float *dimming_to_check);
    float computeGlobalMinimum(float avg_dimming);
    float computeBoundarySolutions(float avg_dimming);
    
    void updateLagrandeMultipliers(float avg_dimming);
};

#endif
