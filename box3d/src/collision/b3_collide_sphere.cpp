
#include "collision/b3_collision.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_cube_shape.hpp"

#include "math/b3_math.hpp"


void b3_collide_spheres(b3Manifold* manifold,
					    const b3SphereShape* sphere_a, 
                        const b3TransformD& xf_a,
					    const b3SphereShape* sphere_b, 
                        const b3TransformD& xf_b) {
    
    manifold->m_point_count = 0;

    b3Vector3d ca = xf_a.transform(sphere_a->get_centroid());
    b3Vector3d cb = xf_b.transform(sphere_b->get_centroid());

    b3Vector3d ab = cb - ca;

    double sq_distance = ab.dot(ab);
    double radius = sphere_a->get_radius() + sphere_b->get_radius();

    if(sq_distance > radius * radius) {
        // two spheres are not collide.
        return;
    }

    manifold->m_type = b3Manifold::e_circles;
    manifold->m_point_count = 1;
    manifold->m_local_normal = ab.normalized();

    double penetration = (b3_sqrt(sq_distance) - radius) / 2.0;
    
    double length = sphere_a->get_radius() + penetration;

    // the collide point on the world frame
    manifold->m_points[0].m_local_point = ca + manifold->m_local_normal * length;
    manifold->m_points[0].id.key = 0;
    manifold->m_penetration = penetration;
}


struct b3SphereCubeFeature {
    int32 m_count = 0;
    b3Vector3d m_axis = b3Vector3d::zero();
};


static void init_feature(const double& value, const double& x, b3SphereCubeFeature& feature, int32 index) {
    if(value < -x) {
        feature.m_axis[index] = -1.0;
        feature.m_count++;
    } else if(value > x) {
        feature.m_axis[index] = 1.0;
        feature.m_count++;
    }
    // -x <= value <= x
    // feature.m_axis_distance[index] = 0
}


static void find_feature(const b3Vector3d& point, const b3CubeShape* cube, b3SphereCubeFeature& feature) {
    init_feature(point.x(), cube->m_h_xyz.x(), feature, 0);
    init_feature(point.y(), cube->m_h_xyz.y(), feature, 1);
    init_feature(point.z(), cube->m_h_xyz.z(), feature, 2);
}


static void find_deep_penetration(const b3Vector3d& sphere_center, b3Manifold* manifold,
                                  const b3Vector3d& cube_h_xyz, const b3TransformD& cube_xf) {

    manifold->m_point_count = 1;
    // TODO: Check this type is useful ?
    // because the points are in the world frame when we construct manifold.
    manifold->m_type = b3Manifold::e_circles;

    b3Vector3d penetration = sphere_center.abs() - cube_h_xyz;
    // penetration < 0
    if(penetration.x() > penetration.y()) {
        // max penetration axis is x.
        if(penetration.x() > penetration.z()) {
            if(sphere_center.x() > 0) {
                manifold->m_local_normal = cube_xf.transform(b3Vector3d(1.0, 0, 0));
            } else {
                manifold->m_local_normal = cube_xf.transform(b3Vector3d(-1.0, 0, 0));
            }

            manifold->m_penetration = penetration.x() / 2;
            double local_point_x = sphere_center.x() + manifold->m_penetration * manifold->m_local_normal.x();
            b3Vector3d local_point(local_point_x, sphere_center.y(), sphere_center.z());
            manifold->m_points[0].m_local_point = cube_xf.transform(local_point);
            manifold->m_points[0].id.key = 0;

            return;
        }
        // max penetration axis is z.
    } else {
        // max penetration axis is y.
        if(penetration.y() > penetration.z()) {
            if(sphere_center.y() > 0) {
                manifold->m_local_normal = cube_xf.transform(b3Vector3d(0, 1.0, 0));
            } else {
                manifold->m_local_normal = cube_xf.transform(b3Vector3d(0, -1.0, 0));
            }

            manifold->m_penetration = penetration.y() / 2;
            double local_point_y = sphere_center.y() + manifold->m_penetration * manifold->m_local_normal.y();
            b3Vector3d local_point(sphere_center.x(), local_point_y, sphere_center.z());
            manifold->m_points[0].m_local_point = cube_xf.transform(local_point);
            manifold->m_points[0].id.key = 0;

            return;
        }
        // max penetration axis is z.
    }

    if(sphere_center.z() > 0) {
        manifold->m_local_normal = cube_xf.transform(b3Vector3d(0, 0, 1.0));
    } else {
        manifold->m_local_normal = cube_xf.transform(b3Vector3d(0, 0, -1.0));
    }

    manifold->m_penetration = penetration.z() / 2;
    double local_point_z = sphere_center.z() + manifold->m_penetration * manifold->m_local_normal.z();
    b3Vector3d local_point(sphere_center.x(), sphere_center.y(), local_point_z);
    manifold->m_points[0].m_local_point = cube_xf.transform(local_point);
    manifold->m_points[0].id.key = 0;
}


static void face_separation(b3Manifold* manifold, const b3Vector3d& axis, const b3Vector3d& sphere_center_local,
                            double total_radius, const b3CubeShape* cube, const b3TransformD& cube_xf) {

    double separation = b3_abs(sphere_center_local.dot(axis));
    if(separation < total_radius) {
        manifold->m_point_count = 1;
        manifold->m_local_normal = cube_xf.transform(axis);
        manifold->m_penetration = (separation - total_radius) / 2.0;
        manifold->m_type = b3Manifold::e_circles;
        manifold->m_points[0].id.key = 0;

        b3Vector3d local_point = sphere_center_local - sphere_center_local.array_dot(axis)
                                 + (cube->m_h_xyz.array_dot(axis))
                                 + manifold->m_penetration * axis;
        manifold->m_points[0].m_local_point = cube_xf.transform(local_point);
    }
}


static void edge_separation(b3Manifold* manifold, const b3Vector3d& axis, const b3Vector3d& sphere_center_local,
                            double total_radius, const b3CubeShape* cube, const b3TransformD& cube_xf) {

    b3Vector3d mid_point = cube->m_h_xyz.array_dot(axis);
    int zero_axis = -1;
    for (int i = 0; i < 3; ++i) {
        if (axis[i] == 0) {
            zero_axis = i;
        }
    }
    b3_assert(zero_axis >= 0 && zero_axis <= 2);

    b3Vector3d point_a = mid_point;
    b3Vector3d point_b = mid_point;
    point_a[zero_axis] -= cube->m_h_xyz[zero_axis];
    point_b[zero_axis] += cube->m_h_xyz[zero_axis];

    b3Vector3d edge_normal = (point_b - point_a).normalized();
    b3Vector3d project_point = point_a + (sphere_center_local - point_a).dot(edge_normal) * edge_normal;
    double separation = (sphere_center_local - project_point).length();

    if (separation < total_radius) {
        manifold->m_point_count = 1;
        manifold->m_type = b3Manifold::e_circles;
        manifold->m_penetration = (separation - total_radius) / 2.0;

        b3Vector3d local_normal = (sphere_center_local - project_point).normalized();
        manifold->m_local_normal = cube_xf.transform(local_normal);

        manifold->m_points[0].id.key = 0;

        b3Vector3d local_point = project_point + manifold->m_penetration * local_normal;
        manifold->m_points[0].m_local_point = cube_xf.transform(local_point);
    }
}


static void vertex_separation(b3Manifold* manifold, const b3Vector3d& axis, const b3Vector3d& sphere_center_local,
                              double total_radius, const b3CubeShape* cube, const b3TransformD& cube_xf) {

    b3Vector3d  point = cube->m_h_xyz.array_dot(axis);
    double separation = (sphere_center_local - point).length();

    if(separation < total_radius) {
        manifold->m_point_count = 1;
        manifold->m_local_normal = cube_xf.transform(point.normalized());
        manifold->m_type = b3Manifold::e_circles;
        manifold->m_penetration = (separation - total_radius) / 2.0;
        manifold->m_points[0].id.key = 0;

        b3Vector3d local_point = point + manifold->m_penetration * point.normalized();
        manifold->m_points[0].m_local_point = cube_xf.transform(local_point);
    }
}


void b3_collide_cube_and_sphere(b3Manifold* manifold,
                                const b3CubeShape* cube_a,
                                const b3TransformD& xf_a,
                                const b3SphereShape* sphere_b,
                                const b3TransformD& xf_b) {
    
    manifold->m_point_count = 0;

    /// Compute the center of sphere position on cube frame
    b3Vector3d sphere_c_local = xf_b.transform(sphere_b->get_centroid());
    sphere_c_local = xf_a.transform_local(sphere_c_local);

    b3SphereCubeFeature feature;
    find_feature(sphere_c_local, cube_a, feature);

    if(feature.m_count == 0) {
        // the center of sphere is in the cube.
        // deep penetration
        // find max penetration
        find_deep_penetration(sphere_c_local, manifold, cube_a->m_h_xyz, xf_a);
        return;
    }

    // shallow penetration
    double total_radius = sphere_b->get_radius() + cube_a->get_radius();

    // b3Vector3d point = cube_a->m_h_xyz.array_dot(feature.m_axis);
    if(feature.m_count == 1) {
        // the closest feature is a face of this cube
        face_separation(manifold, feature.m_axis, sphere_c_local, total_radius, cube_a, xf_a);

    } else if(feature.m_count == 2) {
        // the closest feature is an edge of this cube
        edge_separation(manifold, feature.m_axis, sphere_c_local, total_radius, cube_a, xf_a);

    } else { // feature.m_count == 3
        // the closest feature is a vector of this cube
        vertex_separation(manifold, feature.m_axis, sphere_c_local, total_radius, cube_a, xf_a);
    }

}