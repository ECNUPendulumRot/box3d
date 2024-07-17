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

#ifndef BOX3D_B3_CONTACT_SOLVER_ZHB_HPP
#define BOX3D_B3_CONTACT_SOLVER_ZHB_HPP

#include "common/b3_time_step.hpp"
#include "collision/b3_collision.hpp"
#include "solver/b3_contact_constraint.hpp"

/////////// Forward Delaration ///////////
class b3BlockAllocator;
class b3Contact;
//////////////////////////////////////////

/**
 * @brief Defines the parameters for setting up a contact solver.
 */
struct b3ContactSolverDef
{
    /**
     * @brief The time step information used for simulation.
     */
    b3TimeStep step;
    
    /**
     * @brief Array of pointers to contact objects.
     */
    b3Contact** contacts;
    
    /**
     * @brief The number of contacts.
     */
    int32 count;
    
    /**
     * @brief Array of position vectors for bodies.
     */
    b3Vec3r* ps;
    
    /**
     * @brief Array of orientation quaternions for bodies.
     */
    b3Quatr* qs;
    
    /**
     * @brief Array of linear velocities for bodies.
     */
    b3Vec3r* vs;
    
    /**
     * @brief Array of angular velocities for bodies.
     */
    b3Vec3r* ws;
    
    /**
     * @brief Pointer to the block allocator for memory management.
     */
    b3BlockAllocator* block_allocator;
};


/**
 * @brief Solver for handling contact constraints using the ZHB method.
 */
class b3ContactSolverZHB {
    /**
     * @brief Pointer to the array of position vectors.
     */
    b3Vec3r* m_ps = nullptr;
    
    /**
     * @brief Pointer to the array of orientation quaternions.
     */
    b3Quatr* m_qs = nullptr;
    
    /**
     * @brief Pointer to the array of linear velocities.
     */
    b3Vec3r* m_vs = nullptr;
    
    /**
     * @brief Pointer to the array of angular velocities.
     */
    b3Vec3r* m_ws = nullptr;
    
    /**
     * @brief Pointer to the array of contact objects.
     */
    b3Contact** m_contacts = nullptr;
    
    /**
     * @brief The number of contacts.
     */
    int32 m_count;
    
    /**
     * @brief The time step information used for simulation.
     */
    b3TimeStep m_step;
    
    /**
     * @brief Pointer to the block allocator for memory management.
     */
    b3BlockAllocator* m_block_allocator = nullptr;
    
    /**
     * @brief Pointer to the array of position constraints.
     */
    b3ContactPositionConstraint* m_position_constraints;
    
    /**
     * @brief Pointer to the array of velocity constraints.
     */
    b3ContactVelocityConstraint* m_velocity_constraints;
    
    /**
     * @brief Flag indicating if the solver is waiting for certain conditions.
     */
    int32 m_wait = 0;
    
    /**
     * @brief The current iteration count for solving constraints.
     */
    int32 iteration = 1;

public:
    /**
     * @brief Constructs a b3ContactSolverZHB object using the provided definition.
     * @param def A pointer to the b3ContactSolverDef structure containing solver parameters.
     */
    b3ContactSolverZHB(b3ContactSolverDef* def);

    /**
     * @brief Destructor for the b3ContactSolverZHB class.
     */
    ~b3ContactSolverZHB();

    /**
     * @brief Initializes velocity constraints for the solver.
     */
    void init_velocity_constraints();

    /**
     * @brief Solves the velocity constraints and handles constraint violations.
     * @param violate Reference to a boolean indicating if any constraints are violated.
     * @param propagations Reference to an integer tracking the number of propagations performed.
     */
    void solve_velocity_constraints(bool &violate, int32 &propagations);
};


#endif //BOX3D_B3_CONTACT_SOLVER_ZHB_HPP
