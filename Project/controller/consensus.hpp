#ifndef CONSENSUS_HPP
#define CONSENSUS_HPP

#include <Math.h>
#include "Arduino.h"
#include "util.h"

#define optimization_rho 0.07 //Value teacher used, but we may need to adjust
#define max_iterations 5

class Consensus {
    int current_num_of_iterations = 0;
    float lower_L_bound = -1;
    float local_offset = -1;
    float local_gains[3] = {0};
    float local_cost = -1;
    float lower_actuator_bound = 0; // Lower and Upper dimming values [0, 100] in our case
    float upper_actuator_bound = 100;

    float dimmings[3][3] = {{0}};
    float avg_dimming[3] = {0};
    float lagrange_multipliers[3] = {0};

  public:
    Consensus();
    ~Consensus();
    void Init(float _lower_L_bound, float _local_offset, float _local_gains[3], float _local_cost); //Constructor
    void computeValueToSend( byte my_address, int number_of_addresses, byte nodes_addresses[4]);
    float *computeGlobalMinimum( byte my_address, int number_of_nodes, byte nodes_addresses[4]);
    bool FeasibilityCheck(float dimming_to_check[3], byte my_address, int number_of_nodes);
    float *computeBoundarySolutions( byte my_address, int number_of_nodes, byte nodes_addresses[4]);
    void updateLagrandeMultipliers( byte my_address, int number_of_addresses, byte nodes_addresses[4] );
    void updateAverage(int number_of_addresses);
    float computeCost(float vector_dimming[3], int number_of_nodes, byte my_index);

    float *getDimmings( byte my_address, int number_of_addresses, byte nodes_addresses[4]);
    int getCurrentIteration();

    void updateDimmings( byte my_address, int number_of_addresses, byte nodes_addresses[4], float tmpDimmings[3][3] );
    void incrementIterations();
    float getMyDimming(byte my_address, int number_of_addresses, byte nodes_addresses[4]);
};

#endif
