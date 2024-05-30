
#include "collision/algorithm/b3_sphere_box_collision_algorithm.hpp"

#include "collision/b3_persistent_manifold.hpp"
#include "collision/b3_manifold_result.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_manifold_result.hpp"
#include "dynamics/b3_body.hpp"

#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_cube_shape.hpp"

b3SphereBoxCollisionAlgorithm::b3SphereBoxCollisionAlgorithm(b3Dispatcher* dispatcher) : b3CollisionAlgorithm(dispatcher)
{
    m_manifold = nullptr;
}


b3SphereBoxCollisionAlgorithm::~b3SphereBoxCollisionAlgorithm()
{
    if (m_manifold) {
        m_dispatcher->release_manifold(m_manifold);
    }
}


void b3SphereBoxCollisionAlgorithm::process_collision(
    const b3Fixture *fixtureA, const b3Fixture *fixtureB,
    const b3DispatcherInfo &info, b3PersistentManifold *manifold)
{
    if (m_manifold == nullptr) {
        m_manifold = manifold;
    }

    // bodyA is the box, bodyB is the sphere
    b3Body* bodyA = fixtureA->get_body();
    b3Body* bodyB = fixtureB->get_body();

    b3ManifoldResult result(bodyA, bodyB);
    result.set_persistent_manifold(manifold);

    b3Vec3r sphere_center = bodyB->get_position();

    const b3CubeShape* shapeA = (b3CubeShape*)fixtureA->get_shape();
    const b3SphereShape* shapeB = (b3SphereShape*)fixtureB->get_shape();

    b3Vec3r normal;
    real penetration;
    b3Vec3r point_on_box;

    b3Transformr xfA;
    xfA.set(bodyA->get_position(), bodyA->get_quaternion());

    if (get_sphere_distance(shapeA, xfA, point_on_box, normal, penetration, sphere_center, shapeB->get_radius())) {
        result.add_contact_point(normal, point_on_box, penetration);
    }

    if (manifold->get_contact_point_count() > 0) {
        result.refresh_contact_points();
    }
}


bool b3SphereBoxCollisionAlgorithm::get_sphere_distance(
    const b3CubeShape *shapeA, b3Transformr &xfA, b3Vec3r &point_on_box,
    b3Vec3r &normal, real &penetration, const b3Vec3r &sphere_center, real sphere_radius)
{
    // sphere_center is in world space, transform it to box space
    b3Vec3r local_sphere_center = xfA.transform_local(sphere_center);

    b3Vec3r box_half_xyz = shapeA->m_h_xyz;

    b3Vec3r closest_point;
    closest_point.x = b3_clamp(local_sphere_center.x, -box_half_xyz.x, box_half_xyz.x);
    closest_point.y = b3_clamp(local_sphere_center.y, -box_half_xyz.y, box_half_xyz.y);
    closest_point.z = b3_clamp(local_sphere_center.z, -box_half_xyz.z, box_half_xyz.z);

    real total_radius = sphere_radius + shapeA->get_radius();

    normal = local_sphere_center - closest_point;

    real distance = normal.length2();
    // there is no penetration
    if (distance > total_radius * total_radius) {
        return false;
    }

    if (distance <= b3_real_epsilon) {
        // the closest point is nearly the center of the sphere
        // deep penetration
        distance = find_normal(box_half_xyz, local_sphere_center, normal, closest_point);
    } else {
        // shallow penetration
        distance = b3_sqrt(distance);
        normal = normal / distance;
    }

    // TODO: This is necessary ?
    point_on_box = closest_point + normal * shapeA->get_radius();
    // point_on_box = closest_point;

    penetration = distance - total_radius;

    // transform normal and point back to world space
    normal = xfA.rotation_matrix() * normal;
    point_on_box = xfA.transform(point_on_box);

    return true;
}


real b3SphereBoxCollisionAlgorithm::find_normal(
    const b3Vec3r &box_half_xyz, const b3Vec3r &local_sphere_center,
    b3Vec3r &normal, b3Vec3r &closet_point)
{
    // in the box frame, the possible separate axis is x, y, z, or -x, -y, -z
    real min_distance = b3_real_max;
    int axis;
    // x, y, z
    for (int i = 0; i < 3; i++) {
        real d = box_half_xyz[i] - local_sphere_center[i];
        if (d < min_distance) {
            min_distance = d;
            axis = i;
        }
    }
    // axis must is 0, 1, or 2
    normal.set_zero();
    normal[axis] = 1;
    closet_point = local_sphere_center;
    closet_point[axis] = box_half_xyz[axis];

    axis = -1;
    // -x, -y, -z
    for (int i = 0; i < 3; i++) {
        real d = local_sphere_center[i] + box_half_xyz[i];
        if (d < min_distance) {
            min_distance = d;
            axis = i;
        }
    }

    if (axis != -1) {
        normal.set_zero();
        normal[axis] = -1;
        closet_point = local_sphere_center;
        closet_point[axis] = -box_half_xyz[axis];
    }

    return -min_distance;
}

