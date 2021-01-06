#ifndef CONSENSUS_HPP
#define CONSENSUS_HPP

#include <Math.h>
#include "Arduino.h"
#include "util.h"

#define optimization_rho 0.07 //Value teacher used, but we may need to adjust
#define max_iterations 20
#define lower_actuator_bound 0.0 // Lower and Upper dimming values [0, 100] in our case
#define upper_actuator_bound 100.0
#define tolerance 0.001

class Consensus {
    int current_num_of_iterations = 0;
    float lower_L_bound = -1;
    float local_offset = -1;
    float local_gains[3] = {0};
    float local_cost = -1;

    float dimmings[3][3] = {{0}};
    float avg_dimming[3] = {0};
    float lagrange_multipliers[3] = {0};
    byte my_address = -1;
    int number_of_addresses = -1;
    byte nodes_addresses[4] = {0};
    int number_of_nodes = -1;

  public:
    Consensus();
    ~Consensus();
    void Init(float _lower_L_bound, float _local_offset, float _local_gains[3], float _local_cost, byte _my_address, int _number_of_addresses, byte _nodes_addresses[4]); //Constructor
    void computeValueToSend( );
    float *computeGlobalMinimum( );
    bool FeasibilityCheck(float dimming_to_check[3]);
    float *computeBoundarySolutions();
    void updateLagrandeMultipliers(  );
    void updateAverage();
    float computeCost(float vector_dimming[3], byte my_index);

    float *getDimmings();
    int getCurrentIteration();

    void updateDimmings( float tmpDimmings[3][3] );
    void incrementIterations();
    float getFinalDimming();
};

#endif
