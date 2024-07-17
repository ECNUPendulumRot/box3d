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

#ifndef BOX3D_B3_SOLVER_ZHB_HPP
#define BOX3D_B3_SOLVER_ZHB_HPP


#include "dynamics/b3_transform.hpp"
#include "math/b3_quat.hpp"


/////////// Forward Delaration ///////////

class b3TimeStep;

class b3Contact;

class b3World;

class b3Body;

class b3Island;

class b3ContactVelocityConstraint;
class b3ContactPositionConstraint;

class b3BlockAllocator;

//////////////////////////////////////////

/**
 * @brief  This class is responsible for solving the physical simulation of rigid bodies within an island.
 */
class b3SolverZHB {

    /**
    * @brief  Pointer to an array of pointers to b3Contact objects, representing the contacts in the simulation.
    */
    b3Contact** m_contacts = nullptr;

    /**
    * @brief Number of contacts in the simulation. 
    */
    int32 m_contact_count;

    /**
    * @brief  Number of contacts in the simulation.
    */
    int32 m_body_count;

    /**
    * @brief  Pointer to an array of positions (b3Vec3r) of the bodies.
    */
    b3Vec3r* m_ps = nullptr;

    /**
    * @brief  Pointer to an array of orientations (b3Quatr) of the bodies.
    */
    b3Quatr* m_qs = nullptr;

    /**
    * @brief  Pointer to an array of velocities (b3Vec3r) of the bodies.
    */
    b3Vec3r* m_vs = nullptr;

    /**
    * @brief  Pointer to an array of angular velocities (b3Vec3r) of the bodies.
    */
    b3Vec3r* m_ws = nullptr;

    /**
    * @brief  Pointer to a b3TimeStep object, which contains information about the simulation timestep.
    */
    b3TimeStep* m_timestep = nullptr;
    /**
    * @brief  Pointer to a b3BlockAllocator object, used for memory management.
    */
    b3BlockAllocator* m_block_allocator = nullptr;
    
    /**
    * @brief  Pointer to an array of pointers to b3Body objects, 
    * representing the bodies in the simulation.
    */
    b3Body** m_bodies;

public:
    /**
     * @brief Default constructor.
     */
    b3SolverZHB() = default;

    /**
     * @brief Parameterized constructor: Initializes the solver with a block allocator, an island, and a timestep.
     * @param block_allocator Pointer to a b3BlockAllocator for memory management.
     * @param island Pointer to a b3Island containing the bodies and contacts to be solved.
     * @param step Pointer to a b3TimeStep object containing the timestep information.
     */
    b3SolverZHB(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    /**
     * @brief Initializes the solver with a block allocator, an island, and a timestep.
     * @param block_allocator Pointer to a b3BlockAllocator for memory management.
     * @param island Pointer to a b3Island containing the bodies and contacts to be solved.
     * @param step Pointer to a b3TimeStep object containing the timestep information.
     */
    void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    /**
     * @brief Solves the simulation for a given state.
     * @param allow_sleep Determines whether bodies are allowed to go to sleep (i.e., become inactive if they are at rest).
     * @return Returns the number of iterations performed during the solving process.
     */
    int solve(bool allow_sleep);

    /**
     * @brief Writes the updated states back to the bodies after solving.
     */
    void write_states_back();
    
    /**
     * @brief Destructor: Cleans up any allocated resources.
     */
    ~b3SolverZHB();

};


#endif 
