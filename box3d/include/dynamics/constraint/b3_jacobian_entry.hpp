
#ifndef B3_JACOBIAN_ENTRY_HPP
#define B3_JACOBIAN_ENTRY_HPP

#include "math/b3_mat33.hpp"

class b3JacobianEntry {
public:
    b3JacobianEntry(){};
    //constraint between two different rigidbodies
    b3JacobianEntry(
        const b3Mat33r& world2A,
        const b3Mat33r& world2B,
        const b3Vec3r& rel_pos1, const b3Vec3r& rel_pos2,
        const b3Vec3r& jointAxis,
        const b3Mat33r& inertiaInvA,
        const real massInvA,
        const b3Mat33r& inertiaInvB,
        const real massInvB)
        : m_linearJointAxis(jointAxis)
    {
        m_aJ = world2A * (rel_pos1.cross(m_linearJointAxis));
        m_bJ = world2B * (rel_pos2.cross(-m_linearJointAxis));
        m_0MinvJt = inertiaInvA * m_aJ;
        m_1MinvJt = inertiaInvB * m_bJ;
        m_Adiag = massInvA + m_0MinvJt.dot(m_aJ) + massInvB + m_1MinvJt.dot(m_bJ);

        b3_assert(m_Adiag > real(0.0));
    }

    //angular constraint between two different rigidbodies
    b3JacobianEntry(const b3Vec3r& jointAxis,
                    const b3Mat33r& world2A,
                    const b3Mat33r& world2B,
                    const b3Vec3r& inertiaInvA,
                    const b3Vec3r& inertiaInvB)
        : m_linearJointAxis(b3Vec3r(real(0.), real(0.), real(0.)))
    {
        m_aJ = world2A * jointAxis;
        m_bJ = world2B * -jointAxis;
        m_0MinvJt = inertiaInvA * m_aJ;
        m_1MinvJt = inertiaInvB * m_bJ;
        m_Adiag = m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);

        b3_assert(m_Adiag > real(0.0));
    }

    //angular constraint between two different rigidbodies
    b3JacobianEntry(const b3Vec3r& axisInA,
                    const b3Vec3r& axisInB,
                    const b3Vec3r& inertiaInvA,
                    const b3Vec3r& inertiaInvB)
        : m_linearJointAxis(b3Vec3r(real(0.), real(0.), real(0.))), m_aJ(axisInA), m_bJ(-axisInB)
    {
        m_0MinvJt = inertiaInvA * m_aJ;
        m_1MinvJt = inertiaInvB * m_bJ;
        m_Adiag = m_0MinvJt.dot(m_aJ) + m_1MinvJt.dot(m_bJ);

        b3_assert(m_Adiag > real(0.0));
    }

    //constraint on one rigidbody
    b3JacobianEntry(
        const b3Mat33r& world2A,
        const b3Vec3r& rel_pos1, const b3Vec3r& rel_pos2,
        const b3Vec3r& jointAxis,
        const b3Vec3r& inertiaInvA,
        const real massInvA)
        : m_linearJointAxis(jointAxis)
    {
        m_aJ = world2A * (rel_pos1.cross(jointAxis));
        m_bJ = world2A * (rel_pos2.cross(-jointAxis));
        m_0MinvJt = inertiaInvA * m_aJ;
        m_1MinvJt = b3Vec3r(real(0.), real(0.), real(0.));
        m_Adiag = massInvA + m_0MinvJt.dot(m_aJ);

        b3_assert(m_Adiag > real(0.0));
    }

    real getDiagonal() const { return m_Adiag; }

    // for two constraints on the same rigidbody (for example vehicle friction)
    real getNonDiagonal(const b3JacobianEntry& jacB, const real massInvA) const
    {
        const b3JacobianEntry& jacA = *this;
        real lin = massInvA * jacA.m_linearJointAxis.dot(jacB.m_linearJointAxis);
        real ang = jacA.m_0MinvJt.dot(jacB.m_aJ);
        return lin + ang;
    }

    // for two constraints on sharing two same rigidbodies (for example two contact points between two rigidbodies)
    real getNonDiagonal(const b3JacobianEntry& jacB, const real massInvA, const real massInvB) const
    {
        const b3JacobianEntry& jacA = *this;
        b3Vec3r lin = jacA.m_linearJointAxis * jacB.m_linearJointAxis;
        b3Vec3r ang0 = jacA.m_0MinvJt * jacB.m_aJ;
        b3Vec3r ang1 = jacA.m_1MinvJt * jacB.m_bJ;
        b3Vec3r lin0 = massInvA * lin;
        b3Vec3r lin1 = massInvB * lin;
        b3Vec3r sum = ang0 + ang1 + lin0 + lin1;
        return sum[0] + sum[1] + sum[2];
    }

    real getRelativeVelocity(const b3Vec3r& linvelA, const b3Vec3r& angvelA, const b3Vec3r& linvelB, const b3Vec3r& angvelB)
    {
        b3Vec3r linrel = linvelA - linvelB;
        b3Vec3r angvela = angvelA * m_aJ;
        b3Vec3r angvelb = angvelB * m_bJ;
        linrel = linrel * m_linearJointAxis;
        angvela += angvelb;
        angvela += linrel;
        real rel_vel2 = angvela[0] + angvela[1] + angvela[2];
        return rel_vel2 + b3_real_epsilon;
    }
    //private:

    b3Vec3r m_linearJointAxis;
    b3Vec3r m_aJ;
    b3Vec3r m_bJ;
    b3Vec3r m_0MinvJt;
    b3Vec3r m_1MinvJt;
    //Optimization: can be stored in the w/last component of one of the vectors
    real m_Adiag;
};

#endif