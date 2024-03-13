
#include "collision/b3_collision.hpp"
#include "geometry/b3_plane_shape.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_cube_shape.hpp"
#include "math/b3_math.hpp"


// project the box onto a separation axis called axis
// the axis is in the world frame
static double transform_to_axis(const b3CubeShape& box, const b3TransformD& xf, const b3Vector3d& axis)
{
    const b3Matrix3d& R = xf.m_r_t;
    const b3Vector3d& half_xyz = box.m_h_xyz;

    return half_xyz.x() * b3_abs(axis.dot(R.col(0))) + half_xyz.y() * b3_abs(axis.dot(R.col(1))) + half_xyz.z() * b3_abs(axis.dot(R.col(2)));
}


// check whether two box will overlap under selected axis
// separate cube_B from cube_A
// so the axis is also from cube_A
static void overlap_on_axis(
    const b3TransformD& xf_A_plane,
    const b3CubeShape& cube_B, const b3TransformD& xf_B,
    const b3Vector3d& axis, double& penetration)
{
    // project box onto the axis
    double project_B = transform_to_axis(cube_B, xf_B, axis);

    const b3Vector3d& plane_center = xf_A_plane.linear();
    b3Vector3d to_center = xf_B.linear() - plane_center;

    // set the penetration of current axis and return whether overlapped.
    // penetration is a value that is smaller than zero
    // the larger the value is, the smaller the penetration is.
    double r = to_center.dot(axis);
    penetration = r - project_B;
}


void b3_collide_plane_and_sphere(b3Manifold* manifold,
                                 const b3PlaneShape* plane_a,
                                 const b3TransformD& xf_a,
                                 const b3SphereShape* sphere_b,
                                 const b3TransformD& xf_b) {
    manifold->m_point_count = 0;

    // transform sphere center to plane frame
    b3Vector3d local_center = xf_b.transform(sphere_b->get_centroid());
    local_center = xf_a.transform_local(local_center);

    b3Vector3d nearest_point;
    if(local_center.x() < -plane_a->m_half_width) {
        nearest_point[0] = -plane_a->m_half_width;
    } else if(local_center.x() > plane_a->m_half_width) {
        nearest_point[0] = plane_a->m_half_width;
    } else {
        nearest_point[0] = local_center.x();
    }

    if(local_center.y() < -plane_a->m_half_length) {
        nearest_point[1] = -plane_a->m_half_length;
    } else if(local_center.y() > plane_a->m_half_length) {
        nearest_point[1] = plane_a->m_half_length;
    } else {
        nearest_point[1] = local_center.y();
    }
    double sq_distance = (local_center - nearest_point).length2();
    double radius = sphere_b->get_radius(); //+ plane_a->get_radius();
    if(sq_distance > radius * radius) {
        return;
    }
    manifold->m_point_count = 1;
    manifold->m_local_normal = xf_a.transform(local_center - nearest_point);
    if(manifold->m_local_normal.is_zero()) {
        manifold->m_local_normal = b3Vector3d(0, 0, 1);
    } else {
        manifold->m_local_normal = manifold->m_local_normal.normalized();
    }
    manifold->m_penetration = (b3_sqrt(sq_distance) - radius) / 2.0;
    manifold->m_points[0].m_local_point = xf_a.transform(nearest_point) +
                                          manifold->m_local_normal * manifold->m_penetration;
}


static int32 find_incident_face(
        b3ClipVertex c[4],
        const b3Vector3d & n_p,
        const b3CubeShape* cube2, const b3TransformD& xf2)
{
    int32 count2 = 6;
    const b3Vector3d* normals2 = cube2->m_normals;
    const b3Vector3d* vertices2 = cube2->m_vertices;
    const b3EdgeIndex* edges2 = cube2->m_edges;
    const b3FaceIndex* faces2 = cube2->m_faces;

    // find the incident face on cube2
    // the incident face is the mose anti-parallel face of cube2 to face1
    // so the dot product is the least
    int32 incident_face_index_2 = 0;
    double min_dot = b3_max_double;
    for (int32 i = 0; i < count2; ++i) {
        double dot = n_p.dot(normals2[i]);
        if (dot < min_dot) {
            min_dot = dot;
            incident_face_index_2 = i;
        }
    }

    b3FaceIndex incident_face = faces2[incident_face_index_2];

    for (int32 i = 0; i < 4; ++i) {
        c[i].v = xf2.transform(vertices2[edges2[incident_face.e[i]].v1]);
        c[i].id.cf.type = b3ContactFeature::e_f_p;
        c[i].id.cf.index_1 = (uint8)0;
        c[i].id.cf.index_2 = (uint8)edges2[incident_face.e[i]].v1;
        c[i].id.cf.index_ext  = (uint8)incident_face.e[i];
    }

    return incident_face_index_2;
}


void b3_collide_plane_and_cube(
    b3Manifold* manifold,
    const b3PlaneShape* plane_a,
    const b3TransformD& xf_a,
    const b3CubeShape* cube_b,
    const b3TransformD& xf_b)
{
    manifold->m_point_count = 0;

    double total_radius = cube_b->get_radius() + plane_a->get_radius();

    const b3Vector3d& axis = xf_a.rotation_matrix().col(2);

    double penetration;
    overlap_on_axis(xf_a, *cube_b, xf_b, axis, penetration);

    if (penetration > total_radius) {
        return;
    }

    // the equation of the plane is n * x - n * t = 0;
    // n * t is called front_offset
    double front_offset = axis.dot(xf_a.linear());

    b3ClipVertex incident_vertices[4];
    find_incident_face(incident_vertices, axis, cube_b, xf_b);

    int32 point_count = 0;
    for (int32 i = 0; i < 4; ++i) {

        double separation = axis.dot(incident_vertices[i].v) - front_offset;

        if (separation <= total_radius) {
            b3ManifoldPoint* cp = manifold->m_points + point_count;
            cp->m_local_point = incident_vertices[i].v;
            cp->id = incident_vertices[i].id;

            ++point_count;
        }
    }
    manifold->m_point_count = point_count;
    manifold->m_local_normal = axis;
    manifold->m_penetration = penetration;
}