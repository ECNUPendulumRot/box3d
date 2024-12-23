
#ifndef B3_POINT2_POINT_CONSTRAINT_HPP
#define B3_POINT2_POINT_CONSTRAINT_HPP

#include "dynamics/constraint/b3_constraint_base.hpp"
#include "dynamics/constraint/b3_jacobian_entry.hpp"

class b3Point2PointConstraint : public b3ConstraintBase {

    b3JacobianEntry m_jac[3];

    b3Vec3r m_pivot_inA;
    b3Vec3r m_pivot_inB;

    real m_erp;
    real m_cfm;

public:

    bool m_use_solve_constraint_obsolete;

    b3Point2PointConstraint(b3Body* bodyA, b3Body* bodyB, const b3Vec3r& pivot_inA, const b3Vec3r& pivot_inB, bool collide_connected = false);

    b3Point2PointConstraint(b3Body* bodyA, const b3Vec3r& pivot_inA, bool collide_connected = false);

    virtual void build_jacobian() override;

    virtual void get_info1(b3ConstraintInfo1* info);

    void get_info1_non_virtual(b3ConstraintInfo1* info);

    virtual void get_info2(b3ConstraintInfo2* info);

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