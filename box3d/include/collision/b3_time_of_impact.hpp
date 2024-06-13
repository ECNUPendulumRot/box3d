
#ifndef BOX3D_B3_TIME_OF_IMPACT_HPP
#define BOX3D_B3_TIME_OF_IMPACT_HPP


#include "collision/b3_distance.hpp"
#include "dynamics/b3_body.hpp"


struct b3TOIInput {

    b3DistanceProxy proxy_a;
    b3DistanceProxy proxy_b;
    b3Sweep sweep_a;
    b3Sweep sweep_b;
    real t_max;
};


struct b3TOIOutput
{
    enum State {
        e_unknown,
        e_failed,
        e_overlapped,
        e_touching,
        e_separated
    };

    State state;
    float t;
};


void b3_time_of_impact(b3TOIOutput* output, const b3TOIInput* input);


#endif //BOX3D_B3_TIME_OF_IMPACT_HPP
