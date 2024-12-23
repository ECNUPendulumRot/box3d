
#include "collision/algorithm/b3_plane_cone_collision_algorithm.hpp"

#include "collision/b3_fixture.hpp"
#include "geometry/b3_cone_shape.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_persistent_manifold.hpp"
#include "collision/b3_manifold_result.hpp"

b3PlaneConeCollisionAlgorithm::b3PlaneConeCollisionAlgorithm(b3Dispatcher *dispatcher) : b3CollisionAlgorithm(dispatcher)
{
}

void b3PlaneConeCollisionAlgorithm::process_collision(
    const b3Fixture *fixtureA, const b3Fixture *fixtureB,
    const b3DispatcherInfo &info, b3PersistentManifold* manifold)
{
    b3PlaneShape* plane_a = (b3PlaneShape*)fixtureA->get_shape();
    b3ConeShape* cone_b = (b3ConeShape*)fixtureB->get_shape();

    b3Body* bodyA = fixtureA->get_body();
    b3Body* bodyB = fixtureB->get_body();

    b3Transformr xf_a(bodyA->get_position(), bodyA->get_quaternion());
    b3Transformr xf_b(bodyB->get_position(), bodyB->get_quaternion());

    const b3Mat33r& Ra = xf_a.rotation_matrix();
    const b3Mat33r& Rb = xf_b.rotation_matrix();
    b3Mat33r Rab = Ra.transpose() * Rb;

    real heightB = cone_b->get_height();
    real radiusB = cone_b->get_radius();

    // transform the tip and the base center of the cone in the plane frame
    b3Vec3r tipB = xf_b.position() + cone_b->get_half_height() * Rb.col(2);
    b3Vec3r local_tipB = Ra.transpose() * tipB;

    real total_radius = plane_a->get_radius();

    if (local_tipB.z > heightB + total_radius) {
        return;
    }

    const b3Vec3r* verticesB = cone_b->get_vertices();
    b3Vec3r local_pointsB[B3_CONE_MAX_BASE_POINTS];
    real min_distance = b3_real_max;
    int min_index = -1;
    for (int i = 0; i < B3_CONE_MAX_BASE_POINTS; i++) {
        local_pointsB[i] = xf_b.position() + Rab * verticesB[i];
        if (min_distance > local_pointsB[i].z) {
            min_distance = local_pointsB[i].z;
            min_index = i;
        }
    }

    if (local_tipB.z > total_radius && min_distance > total_radius) {
        return;
    }

    // manifold->m_point_count > 0

    b3Vec3r normal = Ra.col(2);
    // manifold->m_local_normal = Ra.col(2);

    b3ManifoldResult result(bodyA, bodyB, manifold);

    // the tip of the cone is the deepest contact point
    if (Ra.col(2) == Rb.col(2).abs() && local_tipB.z <= total_radius) {
        // real depth = (xf_a.position() - tipB).dot(normal);
        real depth = local_tipB.z - total_radius;
        b3Vec3r pointA = tipB; // - depth * normal;
        result.add_contact_point(normal, pointA, depth);
        return;
    }
    // the base of the cone is parallel to the plane, so the base center of the cone is the contact point
    if (Ra.col(2).abs() == Rb.col(2) && min_distance <= total_radius) {
        b3Vec3r pointB = tipB + Rb * b3Vec3r(0, 0, heightB);
        // real depth = (xf_a.position() - pointB).dot(normal);
        real depth = total_radius - min_distance;
        b3Vec3r pointA = pointB + depth * normal;
        result.add_contact_point(normal, pointA, depth);
        return;
    }

    if (local_tipB.z <= total_radius && min_distance <= total_radius) {
        // real depth = (xf_a.position() - tipB).dot(normal);
        real depth = total_radius - local_tipB.z;
        b3Vec3r pointA = tipB + depth * normal;
        result.add_contact_point(normal, pointA, depth);

        b3Vec3r pointB = Ra * local_pointsB[min_index];
        // depth = (xf_a.position() - pointB).dot(normal);
        depth = total_radius - min_distance;
        pointA = pointB + depth * normal;
        result.add_contact_point(normal, pointA, depth);
//        manifold->m_point_count = 2;
//        manifold->m_points[0].m_local_point = xf_b.position();
//        manifold->m_points[1].m_local_point = Ra * local_pointsB[min_index];
        return;
    }

    // manifold->m_point_count = 2;
    if (local_tipB.z <= total_radius) {
        real depth = total_radius - local_tipB.z;
        // real depth = (xf_a.position() - tipB).dot(normal);
        b3Vec3r pointA = tipB + depth * normal;
        result.add_contact_point(normal, pointA, depth);
        // manifold->m_points[0].m_local_point = xf_b.position();
    } else {
        b3Vec3r pointB = Ra * local_pointsB[min_index];
        // real depth = (xf_a.position() - pointB).dot(normal);
        real depth = total_radius - min_distance;
        b3Vec3r pointA = pointB + depth * normal;
        result.add_contact_point(normal, pointA, depth);
//        manifold->m_points[0].m_local_point = Ra * local_pointsB[min_index];
    }

    // the tip of the cone and a point on the base of the cone, one is above the plane, one is below the plane
//    real x = b3_abs(local_tipB.z / (local_pointsB[min_index].z - local_tipB.z));
//    if (x <= b3_real_epsilon || (1 - x) <= b3_real_epsilon) {
//        // manifold->m_point_count = 1;
//        return;
//    }
//    b3Vec3r dir = local_pointsB[min_index] - local_tipB;
//    b3Vec3r point = local_tipB + x * dir;
//
//    manifold->m_points[1].m_local_point = Ra * point;
}

