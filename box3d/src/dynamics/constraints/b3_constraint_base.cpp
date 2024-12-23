
#include "dynamics/constraint/b3_constraint_base.hpp"

b3ConstraintBase::b3ConstraintBase(b3Body* bodyA, b3Body* bodyB, bool collide_connected) :
    m_bodyA(bodyA),
    m_bodyB(bodyB),
    m_is_enabled(true),
    m_applied_impulse(0.f),
    m_flags(0),
    m_joint_feedback(nullptr),
    m_collide_connected(collide_connected)
{

}

b3ConstraintBase::b3ConstraintBase(b3Body *bodyA, bool collide_connected) :
    m_bodyA(bodyA),
    m_bodyB(get_fixed_body()),
    m_is_enabled(true),
    m_applied_impulse(0.f),
    m_flags(0),
    m_joint_feedback(nullptr),
    m_collide_connected(collide_connected)
{

}

b3Body* b3ConstraintBase::get_fixed_body() {
    // TODO:
    return nullptr;
}