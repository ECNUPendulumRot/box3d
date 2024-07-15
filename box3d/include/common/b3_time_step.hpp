// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#ifndef BOX3D_B3_TIME_STEP_HPP
#define BOX3D_B3_TIME_STEP_HPP

#include "common/b3_types.hpp"
#include "common/b3_common.hpp"

/**
 * @brief provide symbolic names for different numerical integration methods.
 */
enum b3IntegralMethod {
    e_implicit = 0, ///<  Represents an implicit integration method.
    e_verlet = 1 ///< Represents a Verlet integration method.
};

/**
 * @brief The  struct represents parameters related to a time step in a physics or
 * simulation engine.
 */
struct b3TimeStep {

    // time step
    /**
     * @brief Represents the time step duration.
     */
    real m_dt;

    // inverse time step (0 if dt == 0).
    /**
     * @brief Stores the inverse of the time step
     */
    real m_inv_dt;

    // dt * inv_dt0
    /**
     * @brief  Stores the ratio m_dt * m_inv_dt.
     */
    real m_dt_ratio;

    /**
     * @brief Specifies the number of iterations for velocity resolution during the time step.
     */
    int32 m_velocity_iterations;

    /**
     * @brief Specifies the number of iterations for position resolution during the time step.
     */
    int32 m_position_iterations;

    /**
     * @brief Indicates whether warm starting is enabled or not.
     */
    bool m_warm_starting;

    /**
     * @brief Represents an enumeration or class that defines the method used for
     * integrating motion or forces over the time step.
     */
    b3IntegralMethod m_integral_method;

};


#endif //BOX3D_B3_TIME_STEP_HPP
