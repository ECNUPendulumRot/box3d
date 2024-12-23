
#include "collision/b3_collision.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "geometry/b3_cone_shape.hpp"


void b3_collide_plane_and_cone(b3Manifold* manifold,
   const b3PlaneShape* plane_a, const b3Transformr& xf_a,
   const b3ConeShape* cone_b, const b3Transformr& xf_b)
{

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
        manifold->m_point_count = 0;
        return;
    }

    const b3Vec3r* verticesB = cone_b->get_vertices();
    b3Vec3r local_pointsB[B3_CONE_MAX_BASE_POINTS];
    real min_distance = b3_real_max;
    int min_index = -1;
    for (int i = 0; i < B3_CONE_MAX_BASE_POINTS; i++) {
        local_pointsB[i] = local_tipB + Rab * verticesB[i];
        if (min_distance > local_pointsB[i].z) {
            min_distance = local_pointsB[i].z;
            min_index = i;
        }
    }

    if (local_tipB.z > total_radius && min_distance > total_radius) {
        manifold->m_point_count = 0;
        return;
    }

    // manifold->m_point_count > 0
    manifold->m_local_normal = Ra.col(2);

    // the tip of the cone is the deepest contact point
    if (Ra.col(2) == Rb.col(2) && local_tipB.z <= total_radius) {
        manifold->m_point_count = 1;
        manifold->m_points[0].m_local_point = xf_b.position();
        return;
    }
    // the base of the cone is parallel to the plane, so the base center of the cone is the contact point
    if (Ra.col(2).abs() == Rb.col(2).abs() && min_distance <= total_radius) {
        manifold->m_point_count = 1;
        manifold->m_points[0].m_local_point = xf_b.position() + Rb * b3Vec3r(0, 0, heightB);
        return;
    }

    if (local_tipB.z <= total_radius && min_distance <= total_radius) {
        manifold->m_point_count = 2;
        manifold->m_points[0].m_local_point = xf_b.position();
        manifold->m_points[1].m_local_point = Ra * local_pointsB[min_index];
        return;
    }

    manifold->m_point_count = 2;
    if (local_tipB.z <= total_radius) {
        manifold->m_points[0].m_local_point = xf_b.position();
    } else {
        manifold->m_points[0].m_local_point = Ra * local_pointsB[min_index];
    }

    // the tip of the cone and a point on the base of the cone, one is above the plane, one is below the plane
    real x = b3_abs(local_tipB.z / (local_pointsB[min_index].z - local_tipB.z));
    if (x <= b3_real_epsilon || (1 - x) <= b3_real_epsilon) {
        manifold->m_point_count = 1;
        return;
    }
    b3Vec3r dir = local_pointsB[min_index] - local_tipB;
    b3Vec3r point = local_tipB + x * dir;

    manifold->m_points[1].m_local_point = Ra * point;
}