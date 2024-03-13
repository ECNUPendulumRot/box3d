
#include "collision/b3_collision.hpp"

#include "geometry/b3_cube_shape.hpp"

#include <spdlog/spdlog.h>

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
    const b3CubeShape& cube_A, const b3TransformD& xf_A,
    const b3CubeShape& cube_B, const b3TransformD& xf_B,
    const b3Vector3d& axis, double& penetration)
{
    // project two objects onto the axis.
    double project_A = transform_to_axis(cube_A, xf_A, axis);
    double project_B = transform_to_axis(cube_B, xf_B, axis);


    b3Vector3d to_center = xf_B.linear() - xf_A.linear();

    // set the penetration of current axis and return whether overlapped.
    // penetration is a value that is smaller than zero
    // the larger the value is, the smaller the penetration is.
    double r = to_center.dot(axis);
    penetration = r - (project_A + project_B);
}


static void overlap_on_axis_absolute(
        const b3CubeShape& cube_A, const b3TransformD& xf_A,
        const b3CubeShape& cube_B, const b3TransformD& xf_B,
        const b3Vector3d& axis, double& penetration)
{
    // project two objects onto the axis.
    double project_A = transform_to_axis(cube_A, xf_A, axis);
    double project_B = transform_to_axis(cube_B, xf_B, axis);


    b3Vector3d to_center = xf_B.linear() - xf_A.linear();

    // set the penetration of current axis and return whether overlapped.
    // penetration is a value that is smaller than zero
    // the larger the value is, the smaller the penetration is.
    double r = b3_abs(to_center.dot(axis));
    penetration = r - (project_A + project_B);
}



// test face separation of cube_B from cube_A
static double face_separation(
    b3Manifold* manifold,
    const b3CubeShape* cube_A, const b3TransformD& xf_A,
    const b3CubeShape* cube_B, const b3TransformD& xf_B, int32& face_index)
{

    double max_penetration = -b3_max_double;
    int32 best_index;

    b3Matrix3d R_a = xf_A.rotation_matrix_b3();

    for (int32 i = 0; i < 6; ++i) {

        // get the separation normal of cube_A in the world frame.
        const b3Vector3d& n = R_a * cube_A->m_normals[i];

        double penetration = 0.0;
        // calculate the penetration of cube_B from cube_A
        overlap_on_axis(*cube_A, xf_A, *cube_B, xf_B, n, penetration);
        if (penetration > max_penetration) {
            max_penetration = penetration;
            best_index = i;
        }
    }

    face_index = best_index;
    return max_penetration;
}


static double edge_separation(
    b3Manifold* manifold,
    const b3CubeShape* cube_A, const b3TransformD& xf_A,
    const b3CubeShape* cube_B, const b3TransformD& xf_B,
    int32& axis_index_a, int32& axis_index_b)
{
    double max_penetration = -b3_max_double;

    int32 best_axis_index_a;
    int32 best_axis_index_b;

    b3Matrix3d R_a = xf_A.rotation_matrix_b3();
    b3Matrix3d R_b = xf_B.rotation_matrix_b3();

    for (int32 i = 0; i < 3; i++) {

        const b3Vector3d& axis_A = R_a.col(i);

        for (int32 j = 0; j < 3; j++){
            const b3Vector3d& axis_B = R_b.col(j);
            const b3Vector3d& separation_axis = axis_B.cross(axis_A).normalized();
            double penetration;
            overlap_on_axis_absolute(*cube_A, xf_A, *cube_B, xf_B, separation_axis, penetration);

            if (penetration > max_penetration) {
                max_penetration = penetration;
                best_axis_index_a = i;
                best_axis_index_b = j;
            }
        }
    }

    axis_index_a = best_axis_index_a;
    axis_index_b = best_axis_index_b;

    return max_penetration;
}


static int32 b3_find_incident_face(
    b3ClipVertex c[4],
    const b3CubeShape* cube1, const b3TransformD& xf1, int32 face1,
    const b3CubeShape* cube2, const b3TransformD& xf2)
{
    const b3Vector3d* normals1 = cube1->m_normals;

    int32 count2 = 6;

    const b3Vector3d* normals2 = cube2->m_normals;
    const b3Vector3d* vertices2 = cube2->m_vertices;
    const b3EdgeIndex* edges2 = cube2->m_edges;
    const b3FaceIndex* faces2 = cube2->m_faces;

    b3_assert(0 <= face1 && face1 < 6);

    // the normal of face1 in cube2
    // In this way, we do not need to compute normals of cube2 into world frame
    b3Vector3d normal1 = xf1.m_r_t.transpose() * (xf1.m_r_t * normals1[face1]);

    // find the incident face on cube2
    // the incident face is the mose anti-parallel face of cube2 to face1
    // so the dot product is the least
    int32 incident_face_index_2 = 0;
    double min_dot = b3_max_double;
    for (int32 i = 0; i < count2; ++i) {
        double dot = normal1.dot(normals2[i]);
        if (dot < min_dot) {
            min_dot = dot;
            incident_face_index_2 = i;
        }
    }

    b3FaceIndex incident_face = faces2[incident_face_index_2];

    for (int32 i = 0; i < 4; ++i) {
        c[i].v = xf2.transform(vertices2[edges2[incident_face.e[i]].v1]);
        c[i].id.cf.type = b3ContactFeature::e_f_p;
        c[i].id.cf.index_1 = (uint8)face1;
        c[i].id.cf.index_2 = (uint8)edges2[incident_face.e[i]].v1;
        c[i].id.cf.index_ext  = (uint8)incident_face.e[i];
    }

    return incident_face_index_2;
}


// Sutherland-Hodgman clipping.
static int32 b3_clip_segment_to_face(
    b3ClipVertex* v_out, int32& v_out_count,
    const b3ClipVertex* v_in, const int32& v_in_count,
    const b3Vector3d& n, double offset,
    int32 edge_index_clip, int32 incident_face_index)
{
    // calculate all the distances from the vertices to the plane
    // double distance[v_in_count];
    double distance[8];

    for (int32 i = 0; i < v_in_count; i++) {
        double dist = n.dot(v_in[i].v) - offset;
        distance[i] = dist;
    }

    int32 count = 0;
    for (int32 i = 0; i < v_in_count; i++) {

        const int32 next_i = (i + 1) % v_in_count;
        const double& dist_v1 = distance[i];
        const double& dist_v2 = distance[next_i];

        // try to store the origin vertex
        v_out[count].v = v_in[i].v;
        v_out[count].id = v_in[i].id;
        count += (dist_v1 <= 0);

        // get the clipped vertex on this edge at early
        double alpha = dist_v1 / (dist_v1 - dist_v2);
        const b3Vector3d& v1 = v_in[i].v;
        const b3Vector3d& v2 = v_in[next_i].v;

        // check whether the edge needs to be clipped
        // only if the sign of two points on the edge is different needs to be clipped
        uint8 x = (dist_v1 <= 0 && dist_v2 > 0) || (dist_v1 > 0 && dist_v2 <= 0);

        if (x == 0)
            continue;

        v_out[count].v = v1 + alpha * (v2 - v1);

        // old_index_1: the face index of the reference
        uint8 old_index_1 = v_in[i].id.cf.index_1;

        v_out[count].id.cf.index_1 = edge_index_clip;
        v_out[count].id.cf.index_2 = v_out[count].id.cf.index_ext;

        if (v_in[i].id.cf.type == b3ContactFeature::e_e_e &&
            v_in[next_i].id.cf.type == b3ContactFeature::e_e_e) {

            v_out[count].id.cf.type = b3ContactFeature::e_p_f;
            v_out[count].id.cf.index_2 = incident_face_index;
            v_out[count].id.cf.index_ext = old_index_1;

        } else {
            v_out[count].id = v_in[i].id;
            v_out[count].id.cf.type = b3ContactFeature::e_e_e;
        }

        ++count;

    }
    v_out_count = count;
    // TODO
    return 0;
}


static void create_face_contact(
    b3Manifold* manifold,
    const b3CubeShape* cube_A, const b3TransformD& xf_A,
    const b3CubeShape* cube_B, const b3TransformD& xf_B,
    const int32& face_index_A,  const int32& face_index_B,
    const double& separation_A, const double& separation_B,
    const double& total_radius)

{
    const double k_tol = 0.1 * b3_linear_slop;

    const b3CubeShape* cube1;
    const b3CubeShape* cube2;
    b3TransformD xf1, xf2;
    int32 face1;

    if (separation_B > separation_A + k_tol) {
        // we will separate A from B
        cube1 = cube_B;
        cube2 = cube_A;
        xf1 = xf_B;
        xf2 = xf_A;
        face1 = face_index_B;
        manifold->m_type = b3Manifold::e_face_B;
        manifold->m_penetration = separation_B;
    } else {
        // we will separate B from A
        cube1 = cube_A;
        cube2 = cube_B;
        xf1 = xf_A;
        xf2 = xf_B;
        face1 = face_index_A;
        manifold->m_type = b3Manifold::e_face_A;
        manifold->m_penetration = separation_A;
    }

    // separate cube2 from cube1, the separation face on cube1 is face1
    // firstly, we need find the reference face on cube1 and incident face on cube2
    // the reference face is already found, which is face1

    // for cube clipping, there are at most 8 vertices
    // every edge may generate a new vertex in the mid
    b3ClipVertex incident_vertices[4];
    int32 incident_face_index = b3_find_incident_face(incident_vertices, cube1, xf1, face1, cube2, xf2);

    // normal of the reference face in cube1 in world frame.
    const b3Vector3d& n = xf1.m_r_t * cube1->m_normals[face1];

    const b3EdgeIndex* edges1 = cube1->m_edges;
    const b3FaceIndex& faces1 = cube1->m_faces[face1];

    // given a normal n and two vertices c1 and c2,
    // the equation of the plane across c1 and c2 with normal perpendicular to n:
    // nc \dot (x, y, z)^T - nc \dot c1 = 0
    // where nc equals the normalized of c1c2 cross n
    // in the equation, the part nc \dot c1 is called side_offset

    // the equation of the reference plane is:
    // n \dot (x, y, z)^T - n \dot c1 = 0
    // the n \dot c1 part is called front offset
    const b3Vector3d& v1 = xf1.transform(cube1->m_vertices[edges1[faces1.e1].v1]);
    const b3Vector3d& v2 = xf1.transform(cube1->m_vertices[edges1[faces1.e2].v1]);
    const b3Vector3d& v3 = xf1.transform(cube1->m_vertices[edges1[faces1.e3].v1]);
    const b3Vector3d& v4 = xf1.transform(cube1->m_vertices[edges1[faces1.e4].v1]);

    b3Vector3d face_centroid = (v1 + v2 + v3 + v4) / 4.0;

    b3Vector3d tangent1 = (v2 - v1).normalized();
    b3Vector3d tangent2 = (v3 - v2).normalized();
    b3Vector3d tangent3 = (v4 - v3).normalized();
    b3Vector3d tangent4 = (v1 - v4).normalized();

    b3Vector3d nc1 = tangent1.cross(n);
    b3Vector3d nc2 = tangent2.cross(n);
    b3Vector3d nc3 = tangent3.cross(n);
    b3Vector3d nc4 = tangent4.cross(n);

    double side_offset1 = nc1.dot(v1);
    double side_offset2 = nc2.dot(v2);
    double side_offset3 = nc3.dot(v3);
    double side_offset4 = nc4.dot(v4);

    double front_offset = n.dot(v1);

    b3ClipVertex clip_points1[8];
    b3ClipVertex clip_points2[8];
    b3ClipVertex clip_points3[8];
    b3ClipVertex clip_points4[8];

    int32 np;
    b3_clip_segment_to_face(clip_points1, np, incident_vertices, 4, nc1, side_offset1, faces1.e1, incident_face_index);
    b3_clip_segment_to_face(clip_points2, np, clip_points1, np, nc2, side_offset2, faces1.e2, incident_face_index);
    b3_clip_segment_to_face(clip_points3, np, clip_points2, np, nc3, side_offset3, faces1.e3, incident_face_index);
    b3_clip_segment_to_face(clip_points4, np, clip_points3, np, nc4, side_offset4, faces1.e4, incident_face_index);

    // for complicated poligon, the np maybe invalid, but for box, there is no such problem

    int32 point_count = 0;
    manifold->m_local_normal = n;
    manifold->m_local_point = face_centroid;

    for (int32 i = 0; i < np; ++i) {
        double separation = n.dot(clip_points4[i].v) - front_offset;

        if (separation <= total_radius) {
            b3ManifoldPoint* cp = manifold->m_points + point_count;
            cp->m_local_point = clip_points4[i].v;
            cp->id = clip_points4[i].id;

            ++point_count;
        }
    }
    manifold->m_point_count = point_count;
}


double line_segement_separation(
    const b3Vector3d& v1_a, const b3Vector3d& v2_a,
    const b3Vector3d& v1_b, const b3Vector3d& v2_b,
    b3Vector3d& normal, b3Vector3d& point, b3Vector3d& local_point)
{
    const b3Vector3d& d_A = v2_a - v1_a;
    const b3Vector3d& d_B = v2_b - v1_b;

    double a = d_A.dot(d_A);
    double b = d_A.dot(d_B);
    double c = d_B.dot(d_B);

    const b3Vector3d& r = v1_b - v1_a;
    double e = r.dot(d_A);
    double f = r.dot(d_B);

    double m = a * c - b * b;
    double t_n = e * c - b * f;
    double s_n = e * b - a * f;
    double t = b3_clamp(t_n / m, 0.0, 1.0);
    double s = b3_clamp(s_n / m, 0.0, 1.0);

    // because collision is already happened, t and s must be in [0, 1]
    b3Vector3d c_A = v1_a + t * d_A;
    b3Vector3d c_B = v1_b + s * d_B;

    normal = (c_B - c_A).normalized();
    point = c_B;
    local_point = c_A;
    return -(c_B - c_A).length();
}


void create_edge_contact(
    b3Manifold* manifold,
    const b3CubeShape* cube_A, const b3TransformD& xf_A,
    const b3CubeShape* cube_B, const b3TransformD& xf_B,
    const int32& axis_index_A,  const int32& axis_index_B)
{

    double max_penetration = -b3_max_double;
    int32 best_edge_index_a, best_edge_index_b;

    // to find edge-edge contact, we should test all edges pairs of two cubes.
    for (int32 i = 0; i < 12; i++) {

        // get the normal of edge on sphere_A in the world frame.
        const b3Vector3d& v1_a = cube_A->m_vertices[cube_A->m_edges[i].v1];
        const b3Vector3d& v2_a = cube_A->m_vertices[cube_A->m_edges[i].v2];
        const b3Vector3d& e_n_a = (v2_a - v1_a);

        for (int32 j = 0; j < 12; j++) {

            // get the normal of edge on cube_B in the world frame.
            const b3Vector3d& v1_b = cube_B->m_vertices[cube_B->m_edges[j].v1];
            const b3Vector3d& v2_b = cube_B->m_vertices[cube_B->m_edges[j].v2];
            const b3Vector3d& e_n_b = (v2_b - v1_b);

            double edge_A_dot = e_n_a.dot(cube_A->m_normals[axis_index_A]);
            double edge_B_dot = e_n_b.dot(cube_B->m_normals[axis_index_B]);

            // if the edge is not parallel with the axis, just continue
            if (edge_A_dot == 0 || edge_B_dot == 0) {
                continue;
            }

            const b3Vector3d& v1_a_t = xf_A.transform(v1_a);
            const b3Vector3d& v2_a_t = xf_A.transform(v2_a);
            const b3Vector3d& v1_b_t = xf_B.transform(v1_b);
            const b3Vector3d& v2_b_t = xf_B.transform(v2_b);

            b3Vector3d normal, point, local_point;
            double penetration = line_segement_separation(v1_a_t, v2_a_t, v1_b_t, v2_b_t, normal, point, local_point);

            if (penetration > max_penetration) {
                max_penetration = penetration;
                best_edge_index_a = i;
                best_edge_index_b = j;
                manifold->m_local_normal = normal;
                manifold->m_local_point = local_point;
                manifold->m_points[0].m_local_point = point;
            }
        }
    }

    b3ContactID id;
    id.cf.type = b3ContactFeature::e_e_e;
    id.cf.index_1 = (uint8)best_edge_index_a;
    id.cf.index_2 = (uint8)best_edge_index_b;
    id.cf.index_ext = (uint8)best_edge_index_a;

    manifold->m_type = b3Manifold::e_edges;
    manifold->m_point_count = 1;
    manifold->m_points[0].id = id;
    manifold->m_penetration = max_penetration;
}


void b3_collide_cube(
    b3Manifold* manifold,
    const b3CubeShape* cube_A, const b3TransformD& xf_A,
    const b3CubeShape* cube_B, const b3TransformD& xf_B)
{

    manifold->m_point_count = 0;
    double total_radius = cube_A->get_radius() + cube_B->get_radius();

    // firstly separate cube_B from cube_A
    int32 face_index_A;
    double separation_A = face_separation(manifold, cube_A, xf_A, cube_B, xf_B, face_index_A);
    spdlog::log(spdlog::level::info, "separation_A: {}", separation_A);
    if (separation_A > total_radius) {
        return;
    }

    // then separate cube_A from cube_B
    int32 face_index_B;
    double separation_B = face_separation(manifold, cube_B, xf_B, cube_A, xf_A, face_index_B);
    spdlog::log(spdlog::level::info, "separation_B: {}", separation_B);
    if (separation_B > total_radius) {
        return;
    }

    // find edge separation
    int32 axis_index_A, axis_index_B;
    double separation_edge = edge_separation(manifold, cube_A, xf_A, cube_B, xf_B, axis_index_A, axis_index_B);
    spdlog::log(spdlog::level::info, "separation_edge: {}", separation_edge);
    if (separation_edge > total_radius) {
        return;
    }

    // find the best separation axis
    // to avoid parallel situation, we assume that
    // the face separation has more weight than edge separation
    double tol = 0.95;
    bool face_contact_A = separation_A * tol >= separation_edge;
    bool face_contact_B = separation_B * tol >= separation_edge;

    if (face_contact_A || face_contact_B) {
        spdlog::log(spdlog::level::info, "face contact");
        create_face_contact(manifold, cube_A, xf_A, cube_B, xf_B,
                            face_index_A, face_index_B, separation_A, separation_B, total_radius);
    } else {
        spdlog::log(spdlog::level::info, "edge contact");
        create_edge_contact(manifold, cube_A, xf_A, cube_B, xf_B, axis_index_A, axis_index_B);
    }
}




