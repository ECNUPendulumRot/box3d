
#ifndef BOX3D_B3_TIME_STEP_HPP
#define BOX3D_B3_TIME_STEP_HPP

#include "common/b3_types.hpp"


struct b3TimeStep {

    // time step
    real m_dt;

    // inverse time step (0 if dt == 0).
    real m_inv_dt;

    // dt * inv_dt0
    real m_dt_ratio;

    int32 m_velocity_iterations;

    int32 m_position_iterations;

    bool m_warm_starting;

};


#endif //BOX3D_B3_TIME_STEP_HPP
