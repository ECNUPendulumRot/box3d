
#include "collision/b3_collision.hpp"

#include "geometry/b3_cube_shape.hpp"


// project the box onto a separation axis called axis
double transform_to_axis(const box3d::b3CubeShape& box, const b3TransformD& xf, const b3Vector3d& axis)
{
    b3Matrix3d R = xf.rotation_matrix_b3();
    b3Vector3d half_xyz = box.get_half_xyz();

    return half_xyz.x() * b3_abs(axis.dot(R.col(0))) + half_xyz.y() * b3_abs(axis.dot(R.col(1))) + half_xyz.z() * b3_abs(axis.dot(R.col(2)));
}


// check whether two box will overlap under selected axis
bool overlap_on_axis(const box3d::b3CubeShape& cube_A, const b3TransformD& xf_A,
                     const box3d::b3CubeShape& cube_B, const b3TransformD& xf_B,
                     const b3Vector3d& axis, double& penetration)
{
    // project two objects onto the axis.
    double project_A = transform_to_axis(cube_A, xf_A, axis);
    double project_B = transform_to_axis(cube_B, xf_B, axis);


    b3Vector3d to_center = xf_A.linear() - xf_B.linear();

    // set the penetration of current axis and return whether overlapped
    penetration = b3_abs(to_center.dot(axis));

    return (penetration < project_A + project_B);
}


void b3_collide_cube(
        box3d::b3Manifold* manifold,
        const box3d::b3CubeShape* cube_A, const b3TransformD& xf_A,
        const box3d::b3CubeShape* cube_B, const b3TransformD& xf_B)
{

    manifold->m_point_count = 0;
    double totalRadius = cube_A->get_radius() + cube_B->get_radius();

}


// test face seperation of cube_B from cube_A
void face_separation(
    box3d::b3Manifold* manifold,
    const box3d::b3CubeShape* cube_A, const b3TransformD& xf_A,
    const box3d::b3CubeShape* cube_B, const b3TransformD& xf_B, int32 face_index)
{

    double min_penetration = b3_max_double;
    int32 best_index;

    b3Matrix3d R_a = xf_A.rotation_matrix_b3();

    for (int32 i = 0; i < 6; ++i) {

        // get the separation normal of cube_A in the world frame.
        b3Vector3d n = R_a * cube_A->m_normals[i];

        double penetration = 0.0f;
        bool overlap = overlap_on_axis(*cube_A, xf_A, *cube_B, xf_B, n, penetration);

        if (!overlap) {
            return;
        }
    }
}

