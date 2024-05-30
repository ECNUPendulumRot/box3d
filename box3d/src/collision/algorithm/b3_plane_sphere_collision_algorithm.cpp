
#include "collision/algorithm/b3_plane_sphere_collision_algorithm.hpp"
#include "collision/b3_manifold_result.hpp"
#include "collision/b3_persistent_manifold.hpp"

#include "collision/b3_fixture.hpp"
#include "dynamics/b3_body.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_plane_shape.hpp"

#include "spdlog/spdlog.h"

b3PlaneSphereCollisionAlgorithm::b3PlaneSphereCollisionAlgorithm(b3Dispatcher *dispatcher)
    :b3CollisionAlgorithm(dispatcher)
{

}


void b3PlaneSphereCollisionAlgorithm::process_collision(
    const b3Fixture *fixtureA, const b3Fixture *fixtureB,
    const b3DispatcherInfo &info, b3PersistentManifold *manifold)
{
    const b3Body* bodyA = fixtureA->get_body();
    const b3Body* bodyB = fixtureB->get_body();

    b3PlaneShape* shapeA = (b3PlaneShape*)fixtureA->get_shape();
    b3SphereShape* shapeB = (b3SphereShape*)fixtureB->get_shape();

    // transform sphere center to local space of bodyA
    b3Vec3r local_sphere_center = bodyB->get_position();

    b3Transformr xfA;
    xfA.set(bodyA->get_position(), bodyA->get_quaternion());

    local_sphere_center = xfA.transform_local(local_sphere_center);

    // find the closest point on the plane to the sphere center
    b3Vec3r closest_point;

    closest_point.x = b3_clamp(local_sphere_center.x, -shapeA->m_half_width, shapeA->m_half_width);
    closest_point.y = b3_clamp(local_sphere_center.y, -shapeA->m_half_length, shapeA->m_half_length);
    // closest_point.z is default 0

    b3Vec3r normal = local_sphere_center - closest_point;
    real distance2 = normal.length2();

    real total_radius = shapeB->get_radius() + shapeA->get_radius();
    if (distance2 > total_radius * total_radius) {
        // There is no penetration
        manifold->clear_manifold();
        return;
    }

    // TODO: Consider the boundary situation
    if (normal.z <= 0) {
        normal = {0, 0, 1};
    }
    real distance = b3_sqrt(distance2) - total_radius;
    // transform normal to world space
    normal = xfA.transform(normal).normalized();

    b3ManifoldResult result(bodyA, bodyB, manifold);
    result.add_contact_point(normal, closest_point, distance);
    result.refresh_contact_points();
}



