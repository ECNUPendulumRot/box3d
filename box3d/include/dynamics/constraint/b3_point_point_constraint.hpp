
#ifndef B3_POINT2_POINT_CONSTRAINT_HPP
#define B3_POINT2_POINT_CONSTRAINT_HPP

#include "dynamics/constraint/b3_constraint_base.hpp"
#include "dynamics/constraint/b3_jacobian_entry.hpp"

class b3Point2PointConstraint : public b3ConstraintBase {

    b3JacobianEntry m_jac[3];

    b3Vec3r m_pivot_inA;
    b3Vec3r m_pivot_inB;

    b3Vec3r m_pivotA_world;

    real m_erp;
    real m_cfm;

    // 默认阻止bodyB的重力下落
    b3Vec3r m_apply_force_direction = {0, 0, 1};

public:

    bool m_use_solve_constraint_obsolete;

    void set_apply_force_direction(const b3Vec3r& direction) {
        m_apply_force_direction = direction;
    }

    b3Point2PointConstraint(b3Body* bodyA, b3Body* bodyB, const b3Vec3r& pivot_inA, const b3Vec3r& pivot_inB, bool collide_connected = false);

    b3Point2PointConstraint(b3Body* bodyA, const b3Vec3r& pivot_inA, bool collide_connected = false);

    void build_jacobian() override;

    // bodyA is static body, and bodyB is dynamic body
    // bodyB受到外力，bodyA对bodyB存在支持力，沿着某个轴平衡该力。力平衡，且施加力矩
    // p: bodyB的质心。
    void apply_constraint_force_and_torque(b3SolverBody& solver_bodyB) override;

    void get_info1(b3ConstraintInfo1* info) override;

    void get_info1_non_virtual(b3ConstraintInfo1* info);

    void get_info2(b3ConstraintInfo2* info) override;

    void get_info2_non_virtual(b3ConstraintInfo2* info, const b3Transformr& body0_trans, const b3Transformr& body1_trans);

    void set_pivotA(const b3Vec3r& pivotA)
    {
        m_pivot_inA = pivotA;
    }

    void set_pivotB(const b3Vec3r& pivotB)
    {
        m_pivot_inB = pivotB;
    }

    const b3Vec3r& get_pivot_inA() const
    {
        return m_pivot_inA;
    }

    const b3Vec3r& get_pivot_inB() const
    {
        return m_pivot_inB;
    }

};

#endif