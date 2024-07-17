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

#ifndef BOX3D_B3_SOLVER_GR_HPP
#define BOX3D_B3_SOLVER_GR_HPP


#include "dynamics/b3_transform.hpp"
#include "math/b3_quat.hpp"


/////////// Forward Delaration ///////////

/**
 * @brief Forward declaration of the b3TimeStep class.
 */
class b3TimeStep;

/**
 * @brief Forward declaration of the b3Contact class.
 */
class b3Contact;

/**
 * @brief Forward declaration of the b3World class.
 */
class b3World;

/**
 * @brief Forward declaration of the b3Body class.
 */
class b3Body;

/**
 * @brief Forward declaration of the b3Island class.
 */
class b3Island;

/**
 * @brief Forward declaration of the b3ContactVelocityConstraint class.
 */
class b3ContactVelocityConstraint;

/**
 * @brief Forward declaration of the b3ContactPositionConstraint class.
 */
class b3ContactPositionConstraint;

/**
 * @brief Forward declaration of the b3BlockAllocator class.
 */
class b3BlockAllocator;

//////////////////////////////////////////

/**
 * @brief Implements the contact solver using Gauss-Seidel relaxation.
 */
class b3SolverGR {

    /**
     * @brief Pointer to the array of contacts.
     */
    b3Contact** m_contacts = nullptr;

    /**
     * @brief The number of contacts.
     */
    int32 m_contact_count;

    /**
     * @brief The number of bodies.
     */
    int32 m_body_count;

    /**
     * @brief Pointer to the array of body positions.
     */
    b3Vec3r* m_ps = nullptr;

    /**
     * @brief Pointer to the array of body orientations.
     */
    b3Quatr* m_qs = nullptr;

    /**
     * @brief Pointer to the array of body linear velocities.
     */
    b3Vec3r* m_vs = nullptr;

    /**
     * @brief Pointer to the array of body angular velocities.
     */
    b3Vec3r* m_ws = nullptr;

    /**
     * @brief Pointer to the array of contact velocity constraints.
     */
    b3ContactVelocityConstraint* m_velocity_constraints = nullptr;

    /**
     * @brief Pointer to the array of contact position constraints.
     */
    b3ContactPositionConstraint* m_position_constraints = nullptr;

    /**
     * @brief Pointer to the array of violated contact velocity constraints.
     */
    b3ContactVelocityConstraint** m_violated_constraints = nullptr;

    /**
     * @brief The number of violated constraints.
     */
    int32 m_violated_count = 0;

    /**
     * @brief Pointer to the timestep information.
     */
    b3TimeStep* m_timestep = nullptr;

    /**
     * @brief Pointer to the block allocator used for memory management.
     */
    b3BlockAllocator* m_block_allocator = nullptr;

    /**
     * @brief Pointer to the array of bodies.
     */
    b3Body** m_bodies;

public:

    /**
     * @brief Default constructor for the b3SolverGR class.
     */
    b3SolverGR() = default;

    /**
     * @brief Constructs a b3SolverGR object with the specified block allocator, island, and timestep.
     * @param block_allocator Pointer to the block allocator used for memory management.
     * @param island Pointer to the island containing the bodies and contacts.
     * @param step Pointer to the timestep information.
     */
    b3SolverGR(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    /**
     * @brief Initializes the solver with the specified block allocator, island, and timestep.
     * @param block_allocator Pointer to the block allocator used for memory management.
     * @param island Pointer to the island containing the bodies and contacts.
     * @param step Pointer to the timestep information.
     */
    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    /**
     * @brief Solves the velocity constraints.
     * @param velocity_iterations The number of velocity iterations.
     */
    void solve_velocity_constraints(int32 velocity_iterations);

    /**
     * @brief Initializes the velocity constraints.
     */
    void init_velocity_constraints();

    /**
     * @brief Solves the position constraints and updates the positions and velocities of the bodies.
     * @param allow_sleep Boolean indicating whether to allow the bodies to sleep.
     * @return The number of position iterations performed.
     */
    int solve(bool allow_sleep);

    /**
     * @brief Writes the updated states of the bodies back to the island.
     */
    void write_states_back();

    /**
     * @brief Destructor for the b3SolverGR class.
     */
    ~b3SolverGR();

    /**
     * @brief Finds the violated constraints and updates the array of violated constraints.
     */
    void find_violated_constraints();
};


#endif //BOX3D_B3_SOLVER_GR_HPP