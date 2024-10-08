// The MIT License

// Copyright (c) 2024
// Robot Motion and Vision Laboratory at East China Normal University
// Contact: tophill.robotics@gmail.com

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include "collision/b3_collision.hpp"

#include "geometry/b3_cube_shape.hpp"

#include <spdlog/spdlog.h>

/**
 * @brief project the box onto a separation axis called axis, the axis is in the
 * world frame
 * @param box A reference to a b3CubeShape object representing the box to be projected.
 * @param xf A reference to a b3Transr object representing the transformation
 * (including rotation and position) of the box.
 * @param axis A reference to a b3Vec3r object representing the separation axis in
 * the world frame.
 * @return function returns a real value representing the extent of the box along
 * the given separation axis.
 */
static real transform_to_axis(
    const b3CubeShape &box,
    const b3Transr &xf,
    const b3Vec3r &axis)
{
    const b3Mat33r &R = xf.m_r;
    const b3Vec3r &half_xyz = box.m_h_xyz;

    return half_xyz.x * b3_abs(axis.dot(R.col(0))) +
           half_xyz.y * b3_abs(axis.dot(R.col(1))) +
           half_xyz.z * b3_abs(axis.dot(R.col(2)));
}

/**
 * @brief check whether two box will overlap under selected axis, separate cube_B
 * from cube_A, so the axis is also from cube_A
 * @param cube_A A reference to a b3CubeShape object representing the first box (cube A).
 * @param xf_A A reference to a b3Transr object representing the transformation of cube A.
 * @param cube_B A reference to a b3CubeShape object representing the second box (cube B).
 * @param xf_B A reference to a b3Transr object representing the transformation of cube B.
 * @param axis A reference to a b3Vec3r object representing the axis along which the overlap is being checked.
 * @param penetration A reference to a real variable where the computed penetration
 * depth along the specified axis will be stored.
 */
static void overlap_on_axis(
    const b3CubeShape &cube_A, const b3Transr &xf_A,
    const b3CubeShape &cube_B, const b3Transr &xf_B,
    const b3Vec3r &axis, real &penetration)
{
    // project two objects onto the axis.
    real project_A = transform_to_axis(cube_A, xf_A, axis);
    real project_B = transform_to_axis(cube_B, xf_B, axis);

    b3Vec3r to_center = xf_B.position() - xf_A.position();

    // set the penetration of current axis and return whether overlapped.
    // penetration is a value that is smaller than zero
    // the larger the value is, the smaller the penetration is.
    penetration = to_center.dot(axis) - (project_A + project_B);
}

/**
 * @brief The function checks whether two box-shaped objects overlap along a specified axis.
 * @param cube_A A reference to a b3CubeShape object representing the first box
 * @param xf_A  A reference to a b3Transr object representing the transformation of cube A.
 * @param cube_B A reference to a b3CubeShape object representing the second box
 * @param xf_B A reference to a b3Transr object representing the transformation of cube B.
 * @param axis A reference to a b3Vec3r object representing the axis along which the overlap is being checked.
 * @param penetration A reference to a real variable where the computed penetration
 * depth along the specified axis will be stored.
 */
static void overlap_on_axis_absolute(
    const b3CubeShape& cube_A, const b3Transr& xf_A,
    const b3CubeShape& cube_B, const b3Transr& xf_B,
    const b3Vec3r& axis, real &penetration)
{
    // project two objects onto the axis.
    real project_A = transform_to_axis(cube_A, xf_A, axis);
    real project_B = transform_to_axis(cube_B, xf_B, axis);


    b3Vec3r to_center = xf_B.position() - xf_A.position();

    // set the penetration of current axis and return whether overlapped.
    // penetration is a value that is smaller than zero
    // the larger the value is, the smaller the penetration is.
    real r = b3_abs(to_center.dot(axis));
    penetration = r - (project_A + project_B);
}

/**
 * @brief test face separation of cube_B from cube_A
 * @param cube_A A pointer to a b3CubeShape object representing the first box
 * @param xf_A A reference to a b3Transr object representing the transformation of cube A.
 * @param cube_B A pointer to a b3CubeShape object representing the second box
 * @param xf_B A reference to a b3Transr object representing the transformation of cube B.
 * @param face_index A reference to an integer that will be updated with the index of
 * the face on cube_A that has the maximum separation.
 * @return The function returns a value representing the maximum separation (penetration)
 * of cube_B from cube_A along the face normals of cube_A
 */
static real face_separation(
    const b3CubeShape *cube_A, const b3Transr &xf_A,
    const b3CubeShape *cube_B, const b3Transr &xf_B, int32 &face_index)
{
    real max_penetration = -b3_real_max;
    int32 best_index;

    b3Mat33r R_a = xf_A.rotation_matrix();

    for (int32 i = 0; i < 6; ++i) {
        // get the separation normal of cube_A in the world frame.
        const b3Vec3r &n = R_a * cube_A->m_normals[i];

        real penetration = 0.0;
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

/**
 * @brief The edge_separation function calculates the maximum separation (penetration) between
 * the edges of two cubes (cube_A and cube_B).
 * @param cube_A A pointer to a b3CubeShape object representing the first cube (cube A).
 * @param xf_A A reference to a b3Transr object representing the transformation of cube A.
 * @param cube_B A pointer to a b3CubeShape object representing the second cube (cube B).
 * @param xf_B A reference to a b3Transr object representing the transformation of cube B.
 * @param axis_index_a A reference to an integer that will be updated with the index of the
 * edge axis of cube_A that has the maximum separation.
 * @param axis_index_b A reference to an integer that will be updated with the index of the
 * edge axis of cube_B that has the maximum separation.
 * @return The function returns a value representing the maximum separation between the edges
 * of the two cubes along the separation axis.
 */
static real edge_separation(
    const b3CubeShape* cube_A, const b3Transr& xf_A,
    const b3CubeShape* cube_B, const b3Transr& xf_B,
    int32& axis_index_a, int32& axis_index_b)
{
    real max_penetration = -b3_real_max;

    int32 best_axis_index_a;
    int32 best_axis_index_b;

    b3Mat33r R_a = xf_A.rotation_matrix();
    b3Mat33r R_b = xf_B.rotation_matrix();

    for (int32 i = 0; i < 3; i++) {

        const b3Vec3r& axis_A = R_a.col(i);

        for (int32 j = 0; j < 3; j++){
            const b3Vec3r& axis_B = R_b.col(j);
            const b3Vec3r& separation_axis = axis_B.cross(axis_A).normalized();
            real penetration;
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

/**
 * @brief The b3_find_incident_face function determines the face on cube2 that is
 * most likely to be involved in a collision with a given face (face1) on cube1.
 * @param c An array of four b3ClipVertex structures that will be filled with the
 * vertices of the incident face of cube2 in world coordinates.
 * @param cube1 A pointer to a b3CubeShape object representing the first cube (cube1).
 * @param xf1 A reference to a b3Transr object representing the transformation of cube1.
 * @param face1 An integer representing the index of the face on cube1 that is being checked for collision.
 * @param cube2 A pointer to a b3CubeShape object representing the second cube (cube2).
 * @param xf2 A reference to a b3Transr object representing the transformation of cube2.
 * @return The function returns an integer representing the index of the incident
 * face on cube2 that is most anti-parallel to the specified face on cube1.
 */
static int32 b3_find_incident_face(
    b3ClipVertex c[4],
    const b3CubeShape *cube1, const b3Transr &xf1, int32 face1,
    const b3CubeShape *cube2, const b3Transr &xf2)
{
    const b3Vec3r *normals1 = cube1->m_normals;

    int32 count2 = 6;

    const b3Vec3r *normals2 = cube2->m_normals;
    const b3Vec3r *vertices2 = cube2->m_vertices;
    const b3EdgeIndex *edges2 = cube2->m_edges;
    const b3FaceIndex *faces2 = cube2->m_faces;

    b3_assert(0 <= face1 && face1 < 6);

    // the normal of face1 in cube2
    // In this way, we do not need to compute normals of cube2 into world frame
    b3Vec3r normal1 = xf2.m_r.transpose() * (xf1.m_r * normals1[face1]);

    // find the incident face on cube2
    // the incident face is the mose anti-parallel face of cube2 to face1
    // so the dot product is the least
    int32 incident_face_index_2 = 0;
    real min_dot = b3_real_max;
    for (int32 i = 0; i < count2; ++i) {
        real dot = normal1.dot(normals2[i]);
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
        c[i].id.cf.index_ext = (uint8)incident_face.e[i];
    }

    return incident_face_index_2;
}


// Sutherland-Hodgman clipping.
/**
 * @brief The function performs Sutherland-Hodgman clipping to clip a polygon
 * against a plane.
 * @param v_out An output array of b3ClipVertex structures that will hold the
 * vertices of the clipped polygon.
 * @param v_out_count A reference to an integer that will store the count of
 * vertices in the output clipped polygon.
 * @param v_in An input array of b3ClipVertex structures representing the vertices
 * of the original polygon to be clipped.
 * @param v_in_count A reference to an integer that indicates the number of
 * vertices in the input polygon.
 * @param n A reference to a b3Vec3r object representing the normal of the clipping plane.
 * @param offset A number representing the offset of the clipping plane from the
 * origin along its normal.
 * @param edge_index_clip An integer representing the index of the clipping edge.
 * @param incident_face_index An integer representing the index of the incident face.
 * @return The function returns an integer. Although it is currently always returning 0,
 * it could be adapted to return an error code or status in the future.
 */
static int32 b3_clip_segment_to_face(
    b3ClipVertex *v_out, int32 &v_out_count,
    const b3ClipVertex *v_in, const int32 &v_in_count,
    const b3Vec3r &n, real offset,
    int32 edge_index_clip, int32 incident_face_index)
{
    // calculate all the distances from the vertices to the plane
    // double distance[v_in_count];
    real distance[8];

    for (int32 i = 0; i < v_in_count; i++) {
        real dist = n.dot(v_in[i].v) - offset;
        distance[i] = dist;
    }

    int32 count = 0;
    for (int32 i = 0; i < v_in_count; i++) {

	    const int32 next_i = (i + 1) % v_in_count;
	    const real &dist_v1 = distance[i];
	    const real &dist_v2 = distance[next_i];

	    // try to store the origin vertex
	    v_out[count].v = v_in[i].v;
	    v_out[count].id = v_in[i].id;
	    count += (dist_v1 <= 0);

	    // get the clipped vertex on this edge at early
	    real alpha = dist_v1 / (dist_v1 - dist_v2);
	    const b3Vec3r &v1 = v_in[i].v;
	    const b3Vec3r &v2 = v_in[next_i].v;

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

/**
 * @brief The create_face_contact function generates a contact manifold for collision
 * detection between two cube shapes (cuboids) by identifying and processing the
 * contact points on the faces of the cubes.
 * @param manifold  A pointer to a b3Manifold structure that will store the
 * contact information generated by this function.
 * @param cube_A A pointer to the first cube shape involved in the collision.
 * @param xf_A A reference to the transformation of the first cube shape
 * @param cube_B A pointer to the second cube shape involved in the collision.
 * @param xf_B A reference to the transformation of the second cube shape
 * @param face_index_A The index of the face on the first cube shape.
 * @param face_index_B The index of the face on the second cube shape.
 * @param separation_A The separation distance for face A.
 * @param separation_B The separation distance for face B.
 * @param total_radius The combined radius tolerance for determining contact points.
 */
static void create_face_contact(
    b3Manifold *manifold,
    const b3CubeShape *cube_A, const b3Transr &xf_A,
    const b3CubeShape *cube_B, const b3Transr &xf_B,
    const int32 &face_index_A, const int32 &face_index_B,
    const real &separation_A, const real &separation_B,
    const real &total_radius)
{
    const real k_tol = 0.1 * b3_linear_slop;

    // separate cube2 from cube1
    const b3CubeShape *cube1;
    const b3CubeShape *cube2;
    b3Transr xf1, xf2;
    int32 face1;
    real flip = 1.0;

    if (separation_B > separation_A + k_tol) {
        // we will separate m_ from B
        cube1 = cube_B;
        cube2 = cube_A;
        xf1 = xf_B;
        xf2 = xf_A;
        face1 = face_index_B;
        manifold->m_type = b3Manifold::e_face_B;
        flip = -1.0;
    } else {
        // we will separate B from m_
        cube1 = cube_A;
        cube2 = cube_B;
        xf1 = xf_A;
        xf2 = xf_B;
        face1 = face_index_A;
        manifold->m_type = b3Manifold::e_face_A;
    }

    // separate cube2 from cube1, the separation face on cube1 is face1
    // firstly, we need find the reference face on cube1 and incident face on cube2
    // the reference face is already found, which is face1

    // for cube clipping, there are at most 8 vertices
    // every edge may generate a new vertex in the mid
    b3ClipVertex incident_vertices[4];
    int32 incident_face_index = b3_find_incident_face(incident_vertices, cube1, xf1, face1, cube2, xf2);

    // normal of the reference face in cube1 in world frame.
    const b3Vec3r &n = xf1.m_r * cube1->m_normals[face1];

    const b3EdgeIndex *edges1 = cube1->m_edges;
    const b3FaceIndex &faces1 = cube1->m_faces[face1];

    // given a normal n and two vertices c1 and c2,
    // the equation of the plane across c1 and c2 with normal perpendicular to n:
    // nc \dot (x, y, z)^T - nc \dot c1 = 0
    // where nc equals the normalized of c1c2 cross n
    // in the equation, the part nc \dot c1 is called side_offset

    // the equation of the reference plane is:
    // n \dot (x, y, z)^T - n \dot c1 = 0
    // the n \dot c1 part is called front offset
    b3Vec3r v1 = cube1->m_vertices[edges1[faces1.e1].v1];
    b3Vec3r v2 = cube1->m_vertices[edges1[faces1.e2].v1];
    b3Vec3r v3 = cube1->m_vertices[edges1[faces1.e3].v1];
    b3Vec3r v4 = cube1->m_vertices[edges1[faces1.e4].v1];

    b3Vec3r face_centroid = (v1 + v2 + v3 + v4) / real(4.0);

    v1 = xf1.transform(v1);
    v2 = xf1.transform(v2);
    v3 = xf1.transform(v3);
    v4 = xf1.transform(v4);

    b3Vec3r tangent1 = (v2 - v1).normalized();
    b3Vec3r tangent2 = (v3 - v2).normalized();
    b3Vec3r tangent3 = (v4 - v3).normalized();
    b3Vec3r tangent4 = (v1 - v4).normalized();

    b3Vec3r nc1 = tangent1.cross(n);
    b3Vec3r nc2 = tangent2.cross(n);
    b3Vec3r nc3 = tangent3.cross(n);
    b3Vec3r nc4 = tangent4.cross(n);

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
    b3_clip_segment_to_face(clip_points1, np, incident_vertices, 4,
  						    nc1, side_offset1, faces1.e1, incident_face_index);
    b3_clip_segment_to_face(clip_points2, np, clip_points1, np,
  						    nc2, side_offset2, faces1.e2, incident_face_index);
    b3_clip_segment_to_face(clip_points3, np, clip_points2, np,
  						    nc3, side_offset3, faces1.e3, incident_face_index);
    b3_clip_segment_to_face(clip_points4, np, clip_points3, np,
                            nc4, side_offset4, faces1.e4, incident_face_index);

    // for complicated polygon, the np maybe invalid, but for box, there is no such problem

    int32 point_count = 0;
    manifold->m_local_normal = flip * n;
    manifold->m_local_point = face_centroid;

    for (int32 i = 0; i < np; ++i) {
        double separation = n.dot(clip_points4[i].v) - front_offset;

        if (separation <= total_radius) {
            b3ManifoldPoint *cp = manifold->m_points + point_count;
            b3Vec3r& v = clip_points4[i].v;
            // The point of manifold is the mid point
            cp->m_local_point = xf2.transform_local(clip_points4[i].v);
            cp->id = clip_points4[i].id;

            ++point_count;
        }
    }
    manifold->m_point_count = point_count;
}

/**
 * @brief The function calculates the separation between two line segments in 3D
 * space, which are potentially part of two colliding objects.
 * @param v1_a The starting point of the first line segment.
 * @param v2_a The ending point of the first line segment.
 * @param v1_b The starting point of the second line segment.
 * @param v2_b The ending point of the second line segment.
 * @param normal A reference to a vector that will store the normal vector of the separation.
 * @param point A reference to a vector that will store the midpoint of the closest
 * points between the two line segments.
 * @param local_point A reference to a vector that will store the closest point
 * on the first line segment to the second line segment.
 * @return  Returns the negative of the distance between the closest points of the two line segments.
 */
real line_segement_separation(
    const b3Vec3r& v1_a, const b3Vec3r& v2_a,
    const b3Vec3r& v1_b, const b3Vec3r& v2_b,
    b3Vec3r& normal, b3Vec3r& point, b3Vec3r& local_point)
{
    const b3Vec3r& d_A = v2_a - v1_a;
    const b3Vec3r& d_B = v2_b - v1_b;

    real a = d_A.dot(d_A);
    real b = d_A.dot(d_B);
    real c = d_B.dot(d_B);

    const b3Vec3r& r = v1_b - v1_a;
    real e = r.dot(d_A);
    real f = r.dot(d_B);

    real m = a * c - b * b;
    real t_n = e * c - b * f;
    real s_n = e * b - a * f;

    real t = t_n / m;
    real s = s_n / m;

    // because collision is already happened,
    // there must exist t and s that are in [0.0, 1.0]
    // so if one of t or s is invalid, this is not a valid edge-edge separation
    if (t < 0.0 || 1.0 < t || s < 0.0 || 1.0 < s) {
        return -b3_real_max;
    }

    b3Vec3r c_A = v1_a + t * d_A;
    b3Vec3r c_B = v1_b + s * d_B;

    normal = (c_B - c_A).normalized();
    point = (c_A + c_B) / real(2.0);
    local_point = c_A;
    return -(c_B - c_A).length();
}

/**
 * @brief The function aims to detect and create contact information for the
 * collision between the edges of two cubes
 * @param manifold A pointer to a manifold structure that will store the contact information.
 * @param cube_A A pointer to the first cube shape.
 * @param xf_A The transformation of the first cube.
 * @param cube_B A pointer to the second cube shape.
 * @param xf_B The transformation of the second cube.
 * @param axis_index_A The index of the axis in the identity matrix to be used as
 * the reference for edges on the first cube.
 * @param axis_index_B The index of the axis in the identity matrix to be used as
 * the reference for edges on the second cube.
 * @param separation The separation distance between the cubes to be checked.
 * @return Returns true if a valid edge contact is found and created; otherwise, returns false.
 */
bool create_edge_contact(
    b3Manifold* manifold,
    const b3CubeShape* cube_A, const b3Transr& xf_A,
    const b3CubeShape* cube_B, const b3Transr& xf_B,
    const int32& axis_index_A, const int32& axis_index_B,
    const real &separation)
{
    if (b3_abs(separation - 0.015) < 0.001) {
        int32 i = 0;
    }

    real max_penetration = -b3_real_max;
    int32 best_edge_index_a, best_edge_index_b;
    real max_diff = b3_real_max;

    const b3Mat33r& I = b3Mat33r::identity();

    const b3Vec3r& axis_A = I.col(axis_index_A);
    const b3Vec3r& axis_B = I.col(axis_index_B);

    // to find edge-edge contact, we should test all edges pairs of two cubes.
    for (int32 i = 0; i < 12; i++) {

        // get the normal of edge on sphere_A in the world frame.
        const b3Vec3r& v1_a = cube_A->m_vertices[cube_A->m_edges[i].v1];
        const b3Vec3r& v2_a = cube_A->m_vertices[cube_A->m_edges[i].v2];
        const b3Vec3r& e_n_a = (v2_a - v1_a);

        if (b3_close_to_zero(e_n_a.dot(axis_A))) {
            continue;
        }

        for (int32 j = 0; j < 12; j++) {

            // get the normal of edge on cube_B in the world frame.
            const b3Vec3r& v1_b = cube_B->m_vertices[cube_B->m_edges[j].v1];
            const b3Vec3r& v2_b = cube_B->m_vertices[cube_B->m_edges[j].v2];
            const b3Vec3r& e_n_b = (v2_b - v1_b);

            if (b3_close_to_zero(e_n_b.dot(axis_B))) {
                continue;
            }

            const b3Vec3r& v1_a_t = xf_A.transform(v1_a);
            const b3Vec3r& v2_a_t = xf_A.transform(v2_a);
            const b3Vec3r& v1_b_t = xf_B.transform(v1_b);
            const b3Vec3r& v2_b_t = xf_B.transform(v2_b);

            b3Vec3r normal, point, local_point;
            real penetration = line_segement_separation(v1_a_t, v2_a_t, v1_b_t, v2_b_t, normal, point, local_point);

            real diff = b3_abs(separation - penetration);
            if (diff < max_diff) {
                max_diff = diff;
                best_edge_index_a = i;
                best_edge_index_b = j;
                manifold->m_local_normal = normal;
                manifold->m_local_point = local_point;
                manifold->m_points[0].m_local_point = point;
            }
        }
    }

    if (max_diff == b3_real_max) {
        return false;
    }

    b3ContactID id;
    id.cf.type = b3ContactFeature::e_e_e;
    id.cf.index_1 = (uint8)best_edge_index_a;
    id.cf.index_2 = (uint8)best_edge_index_b;
    id.cf.index_ext = (uint8)best_edge_index_a;

    manifold->m_type = b3Manifold::e_edges;
    manifold->m_point_count = 1;
    manifold->m_points[0].id = id;

    return true;
}

/**
 * @brief Compute the collision manifold between two cubes
 * @param manifold Pointer to the b3Manifold structure for storing collision details between cubes
 * @param cube_A Pointer to the first cube shape involved in the collision
 * @param xf_A The transformation information of the first cube, including position and orientation
 * @param cube_B Pointer to the second cube shape involved in the collision
 * @param xf_B Transformation information for the second cube, including position and orientation.
 */
void b3_collide_cube(
    b3Manifold *manifold,
    const b3CubeShape *cube_A, const b3Transr &xf_A,
    const b3CubeShape *cube_B, const b3Transr &xf_B)
{
    manifold->m_point_count = 0;
    real total_radius = cube_A->get_radius() + cube_B->get_radius();

    // firstly separate cube_B from cube_A
    int32 face_index_A;
    real separation_A = face_separation(cube_A, xf_A, cube_B, xf_B, face_index_A);
    if (separation_A > total_radius) {
        return;
    }

    // then separate cube_A from cube_B
    int32 face_index_B;
    real separation_B = face_separation(cube_B, xf_B, cube_A, xf_A, face_index_B);
    if (separation_B > total_radius) {
        return;
    }

    // find edge separation
    int32 axis_index_A, axis_index_B;
    real separation_edge = edge_separation(cube_A, xf_A, cube_B, xf_B, axis_index_A, axis_index_B);
    if (separation_edge > total_radius) {
        return;
    }

    // find the best separation axis
    // to avoid parallel situation, we assume that
    // the face separation has more weight than edge separation
    real tol = real(0.1) * b3_linear_slop;
    bool face_contact_A = (separation_A + tol) >= separation_edge;
    bool face_contact_B = (separation_B + tol) >= separation_edge;

    bool edge_succeed = true;
    if (!face_contact_A && !face_contact_B) {
        edge_succeed = create_edge_contact(manifold, cube_A, xf_A, cube_B, xf_B, axis_index_A, axis_index_B, separation_edge);
    } else if (edge_succeed == false || face_contact_A || face_contact_B) {
        create_face_contact(manifold, cube_A, xf_A, cube_B, xf_B,
                            face_index_A, face_index_B, separation_A, separation_B, total_radius);
    }
}

