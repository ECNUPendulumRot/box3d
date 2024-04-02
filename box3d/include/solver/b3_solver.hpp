
#ifndef BOX3D_B3_SOLVER_HPP
#define BOX3D_B3_SOLVER_HPP

#include "dynamics/b3_transform.hpp"
#include "math/b3_quaternion.hpp"
#include "solver/b3_contact_constraint.hpp"
#include "common/b3_time_step.hpp"

/////////// Forward Delaration ///////////

class b3TimeStep;

class b3Contact;

class b3World;

class b3Body;

class b3Island;

class b3AffineContactVelocityConstraint;

class b3BlockAllocator;

//////////////////////////////////////////

void b3_get_two_tangent_bases(const b3Vector3r& normal, b3Vector3r& t1, b3Vector3r& t2);


class b3Solver {

protected:

    b3Contact** m_contacts = nullptr;
    int32 m_contact_count;

    int32 m_body_count;

    b3TimeStep* m_timestep = nullptr;

    b3BlockAllocator* m_block_allocator = nullptr;

    b3Body** m_bodies;

    b3IntegralMethod m_method;

    b3AffineContactVelocityConstraint* m_avc = nullptr;

    Eigen::Vector<real, 12>* m_affine_qs = nullptr;

    Eigen::Vector<real, 12>* m_affine_q_dots = nullptr;

    /**
     * @brief write the velocity and position back to bodies.
     */
    void write_states_back();

public:

    b3Solver() = default;

    //b3Solver(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    virtual void init(b3BlockAllocator* block_allocator, b3Island* island, b3TimeStep* step);

    /**
     * @breif solver all constraints in the island.
     */
    virtual int solve() {
        return 0;
    };

    virtual ~b3Solver() = default;

    void clear();

};




#endif //BOX3D_B3_SOLVER_HPP