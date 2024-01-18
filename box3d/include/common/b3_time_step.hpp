
#ifndef BOX3D_B3_TIME_STEP_HPP
#define BOX3D_B3_TIME_STEP_HPP

#include "common/b3_types.hpp"


namespace box3d {

    struct b3TimeStep;

}

struct box3d::b3TimeStep {

    // time step
    double m_dt;

    // inverse time step (0 if dt == 0).
    double m_inv_dt;

    // dt * inv_dt0
    double m_dt_ratio;

    int32 m_velocity_iterations;

    int32 m_position_iterations;

    bool m_warm_starting;

};

#endif //BOX3D_B3_TIME_STEP_HPP
