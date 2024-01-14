
#include "collision/b3_collision.hpp"

#include "geometry/b3_cube_shape.hpp"


void b3_collide_cube(
    box3d::b3Manifold* manifold,
    const box3d::b3CubeShape* cube_A, const b3TransformD& xf_A,
    const box3d::b3CubeShape* cube_B, const b3TransformD& xf_B)
{



}


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
                     const b3Vector3d& axis)
{
    // project two objects onto the axis.
    double project_A = transform_to_axis(cube_A, xf_A, axis);
    double project_B = transform_to_axis(cube_B, xf_B, axis);


    b3Vector3d to_center = xf_A.linear() - xf_B.linear();
    double d = b3_abs(to_center.dot(axis));
    return (d < project_A + project_B);
}