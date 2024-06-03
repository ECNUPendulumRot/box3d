
#include "collision/b3_manifold_result.hpp"

#include "collision/b3_persistent_manifold.hpp"
#include "dynamics/b3_body.hpp"


b3ManifoldResult::b3ManifoldResult(const b3Body *bodyA, const b3Body *bodyB, b3PersistentManifold* manifold)
    : m_bodyA(bodyA), m_bodyB(bodyB), m_manifold(manifold)
{
}


void b3ManifoldResult::add_contact_point(const b3Vec3r &normal, const b3Vec3r &pointA, real depth)
{
    if (depth > m_manifold->get_contact_breaking_threshold()) {
        return;
    }

    b3Vec3r pointB = pointA + normal * depth;

    b3Transformr xfA, xfB;
    xfA.set(m_bodyA->get_position(), m_bodyA->get_quaternion());
    xfB.set(m_bodyB->get_position(), m_bodyB->get_quaternion());

    b3Vec3r localA = xfA.transform_local(pointA);
    b3Vec3r localB = xfB.transform_local(pointB);

    b3PersistentManifoldPoint new_manifold_point(localA, localB, normal, depth);
    new_manifold_point.m_position_world_on_A = pointA;
    new_manifold_point.m_position_world_on_B = pointB;

    // TODO: calculate friction and restitution, we can do this in persistent manifold,
    // TODO: so we don't need to calculate it for each contact point

    // get two tangent direction of the contact point
    b3_plane_space(new_manifold_point.m_normal_world_on_A, new_manifold_point.m_tangent_direction1, new_manifold_point.m_tangent_direction2);

    int insert_index = m_manifold->get_cache_entry(new_manifold_point);

    if (insert_index >= 0) {
        m_manifold->replace_contact_point(new_manifold_point, insert_index);
    }  else {
        m_manifold->add_manifold_point(new_manifold_point);
    }
}


void b3ManifoldResult::refresh_contact_points()
{
    if (m_manifold->get_contact_point_count() == 0) {
        return;
    }

    // TODO: if we need to add body swapped

    b3Transformr xfA, xfB;
    xfA.set(m_bodyA->get_position(), m_bodyA->get_quaternion());
    xfB.set(m_bodyB->get_position(), m_bodyB->get_quaternion());

    m_manifold->refresh_contact_points(xfA, xfB);
}
