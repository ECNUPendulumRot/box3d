
#include "collision/algorithm/b3_plane_box_collision_algorithm.hpp"
#include "collision/b3_persistent_manifold.hpp"
#include "collision/b3_manifold_result.hpp"

#include "collision/b3_fixture.hpp"
#include "dynamics/b3_body.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "geometry/b3_cube_shape.hpp"

b3PlaneBoxCollisionAlgorithm::b3PlaneBoxCollisionAlgorithm(b3Dispatcher *dispatcher)
    : b3CollisionAlgorithm(dispatcher)
{

}


void b3PlaneBoxCollisionAlgorithm::process_collision(
    const b3Fixture *fixtureA, const b3Fixture *fixtureB,
    const b3DispatcherInfo &info, b3PersistentManifold *manifold)
{
    b3Body* bodyA = fixtureA->get_body();
    b3Body* bodyB = fixtureB->get_body();

    b3PlaneShape* shapeA = (b3PlaneShape*)fixtureA->get_shape();
    b3CubeShape* shapeB = (b3CubeShape*)fixtureB->get_shape();

    // get the normal of the plane in the world frame
    const b3Vec3r& axis = bodyA->get_quaternion().rotation_matrix().col(2);

    b3Transformr xfB;
    xfB.set(bodyB->get_position(), bodyB->get_quaternion());

    // project box onto the axis
    const b3Mat33r& Rb = xfB.rotation_matrix();
    const b3Vec3r& half_xyzB = shapeB->m_h_xyz;
    real projectB = half_xyzB.x * b3_abs(axis.dot(Rb.col(0))) +
                    half_xyzB.y * b3_abs(axis.dot(Rb.col(1))) +
                    half_xyzB.z * b3_abs(axis.dot(Rb.col(2)));

    b3Vec3r to_center = xfB.position() - bodyA->get_position();
    real penetration = to_center.dot(axis) - projectB;

    real total_radius = shapeA->get_radius() + shapeB->get_radius();

    if (penetration > total_radius) {
        manifold->clear_manifold();
        return;
    }

    b3ManifoldResult result(bodyA, bodyB, manifold);

    b3Vec3r incident_vertices[4];
    int count2 = 6;
    const b3Vec3r* normal2 = shapeB->m_normals;
    const b3Vec3r* vertices2 = shapeB->m_vertices;
    const b3EdgeIndex* edges2 = shapeB->m_edges;
    const b3FaceIndex* face2 = shapeB->m_faces;

    // transform axis to box local space
    const b3Vec3r& local_axis = Rb.transpose() * axis;

    // find the incident face on box
    // the incident face is the most anti-parallel face of box to face1
    // so the dot product is the least
    int incident_face_index = 0;
    real min_dot = b3_real_max;
    for (int i = 0; i < count2; i++) {
        real dot = local_axis.dot(normal2[i]);
        if (dot < min_dot) {
            min_dot = dot;
            incident_face_index = i;
        }
    }

    b3FaceIndex incident_face = face2[incident_face_index];

    for (int i = 0; i < 4; i++) {
        incident_vertices[i] = xfB.transform(vertices2[edges2[incident_face.e[i]].v1]);
    }

    // the equation of the plane is n * x - n * t = 0
    // n * t is called front_offset
    real front_offset = axis.dot(bodyA->get_position());
    for (int i = 0; i < 4; i++) {
        real separation = axis.dot(incident_vertices[i]) - front_offset - total_radius;
        if (separation < 0) {
            b3Vec3r point = incident_vertices[i] - separation * axis;

            result.add_contact_point(axis, point, separation);
        }
    }
    result.refresh_contact_points();
}

