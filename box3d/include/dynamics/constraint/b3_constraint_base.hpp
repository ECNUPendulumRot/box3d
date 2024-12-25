
#ifndef B3_CONSTRAINT_BASE_HPP
#define B3_CONSTRAINT_BASE_HPP

#include "dynamics/b3_body.hpp"


class b3SolverBody;
class b3SolverConstraint;

struct b3JointFeedback {
    b3Vec3r m_applied_force_bodyA;
    b3Vec3r m_applied_torque_bodyA;
    b3Vec3r m_applied_force_bodyB;
    b3Vec3r m_applied_torque_bodyB;
};

class b3ConstraintBase {

protected:

    b3Body* m_bodyA;
    b3Body* m_bodyB;
    real m_applied_impulse;
    b3JointFeedback* m_joint_feedback;

    uint32 m_flags = 0;

    bool m_is_enabled;

    bool m_collide_connected = false;

public:

    virtual ~b3ConstraintBase() {}
    b3ConstraintBase(b3Body* bodyA, b3Body* bodyB, bool collide_connected = false);
    b3ConstraintBase(b3Body* bodyA, bool collide_connected = false);

    enum {
        e_island = 1
    };

    struct b3ConstraintInfo1 {
        int32 m_num_constraint_rows, nub;
    };

    struct b3ConstraintInfo2 {
        // integrator parameters: frames per second (1/stepsize), default error reduction parameter (0..1).
        real fps, erp;

        // for the first and second body, pointers to two (linear and angular)
        // n*3 jacobian sub matrices, stored by rows. these matrices will have
        // been initialized to 0 on entry. if the second body is zero then the
        // J2xx pointers may be 0.
        // real *m_J1linear_axis, *m_J1angular_axis, *m_J2linear_axis, *m_J2angular_axis;
        b3SolverConstraint* m_solver_constraint;

        // elements to jump from one row to the next in J's
        int rowskip;

        // right hand sides of the equation J*v = c + cfm * lambda. cfm is the
        // "constraint force mixing" vector. c is set to zero on entry, cfm is
        // set to a constant value (typically very small or zero) value on entry.

        // number of solver iterations
        int m_num_iterations;

        //damping of the velocity
        real m_damping;
    };

    bool get_collide_connected() const {
        return m_collide_connected;
    }

    virtual void apply_constraint_force_and_torque(b3SolverBody& solver_bodyB) {}

    virtual void build_jacobian() {}

    virtual void get_info1(b3ConstraintInfo1* info) = 0;

    virtual void get_info2(b3ConstraintInfo2* info) = 0;

    virtual void solve_constraint_obsolete(b3SolverBody& bodyA, b3SolverBody& bodyB, real dt) {}

    static b3Body* get_fixed_body();

    void internal_set_applied_impulse(real impulse) {
        m_applied_impulse = impulse;
    }


    bool is_enabled() const {
        return m_is_enabled;
    }

    void set_enabled(bool enabled) {
        m_is_enabled = enabled;
    }

    bool test_flag(uint32 flag) const {
        return m_flags & flag;
    }

    void add_flag(uint32 flag) {
        m_flags |= flag;
    }

    void remove_flag(uint32 flag) {
        m_flags &= ~flag;
    }

    b3Body* get_bodyA() const {
        return m_bodyA;
    }

    b3Body* get_bodyB() const {
        return m_bodyB;
    }
};

#endif