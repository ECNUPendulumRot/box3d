
#include "collision/b3_collision.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "geometry/b3_cone_shape.hpp"


void b3_collide_plane_and_cone(b3Manifold* manifold,
   const b3PlaneShape* plane_a, const b3Transformr& xf_a,
   const b3ConeShape* cone_b, const b3Transformr& xf_b)
{

    const b3Mat33r& Ra = xf_a.rotation_matrix();
    const b3Mat33r& Rb = xf_b.rotation_matrix();

    real heightB = cone_b->get_height();
    real radiusB = cone_b->get_radius();

    // transform the tip and the base center of the cone in the plane frame
    b3Vec3r local_tipB = Ra.transpose() * xf_b.position();
    b3Vec3r local_base_centerB = Ra.transpose() * (xf_b.position() + Rb.col(2) * heightB);

    b3Vec3r centerA_to_tip = local_tipB - xf_a.position();
    b3Vec3r centerA_to_base_center = local_base_centerB - xf_a.position();

    // don't care x, and y
    if (centerA_to_tip.z > 0 && centerA_to_base_center.z > radiusB) {
        // there is no penetration
        manifold->m_point_count = 0;
        return;
    }

    manifold->m_local_normal = Ra.col(2);

    if (centerA_to_tip.z < b3_real_epsilon && centerA_to_base_center.z > radiusB) {
        manifold->m_point_count = 1;
        manifold->m_points[0].m_local_point = xf_b.position();
        return;
    }

    // the cone falls on its side to the ground
    // tan(theta) = radiusB / heightB
    // r = (center_to_base_center - center_to_tip).z
    // h = sqrt((center_to_base_center.x - center_to_tip.x)^2 + (center_to_base_center.y - center_to_tip.y)^2)
    // if radiusB / heightB == r / h ==> radiusB * h - heightB * r = 0

    b3Vec3r tip_to_base_center = centerA_to_base_center - centerA_to_tip;
    real r = tip_to_base_center.z;
    real h = b3_sqrt(tip_to_base_center.x * tip_to_base_center.x + tip_to_base_center.y * tip_to_base_center.y);
    if (b3_abs(radiusB * h - heightB * r) < b3_real_epsilon) {
        // two contact points
        manifold->m_point_count = 2;
        // the world position of the tip of the cone
        b3Vec3r point1 = xf_b.position();
        tip_to_base_center.z = 0;
        tip_to_base_center.normalized();
        b3Vec3r point2 = local_tipB + b3_sqrt(radiusB * radiusB + heightB * heightB) * tip_to_base_center;
        point2 = Ra * point2;

        manifold->m_points[0].m_local_point = point1;
        manifold->m_points[1].m_local_point = point2;

    }

    // The contact surface is an ellipse，so generate 4 contact points

    manifold->m_point_count = 4;
    b3Mat33r Rab = Ra.transpose() * Rb;

    static const b3Vec3r search_dirs[4] = {
        b3Vec3r(1, 0, 0),
        b3Vec3r(0, 1, 0),
        b3Vec3r(-1, 0, 0),
        b3Vec3r(0, -1, 0)
    };

    // TODO: reconsider how to generate the contact points
    for (int i = 0; i < 4; i++) {
        b3Vec3r dir = (tip_to_base_center + radiusB * (Rab * search_dirs[i])).normalized();
        // remember, these vector are in the plane frame.
        // tip.z + x * dir.z = 0 ===> x = -tip.z / dir.z
        if (b3_abs(dir.z) > b3_real_epsilon) {
            real x = -local_tipB.z / dir.z;

        }
    }

}