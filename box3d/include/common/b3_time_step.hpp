
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

    int32 m_iterations = 8;

    int32 m_position_iterations = 8;

    bool m_warm_starting;
    // [0, 1]
    real m_warm_starting_factor = 0.85;

    // 过渡松弛因子
    real m_sor = 1.0;
    // 允许的接触最小误差
    real m_erp = 0.2;
    // 刚度因子（maybe）
    real m_global_cfm = 0.0;

    real m_restitution_velocity_threshold = 0.2;
    real m_least_squares_residual_threshold = 0.f;

    bool m_solve_use_2_friction_directions = false;

    real m_split_impulse_penetration_threshold = -0.04;
};


#endif //BOX3D_B3_TIME_STEP_HPP
