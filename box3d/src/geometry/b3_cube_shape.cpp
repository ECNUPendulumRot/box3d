
#include "geometry/b3_cube_shape.hpp"
#include "common/b3_common.hpp"
#include "common/b3_allocator.hpp"


box3d::b3CubeShape::b3CubeShape()
{
    m_radius = b3_polygon_radius;
}


void box3d::b3CubeShape::set_as_box(double hx, double hy, double hz)
{
    b3_assert(hx > 0.0f);
    b3_assert(hy > 0.0f);
    b3_assert(hz > 0.0f);

    m_centroid.set_zero();

    m_vertices[0].set(hx , -hy, -hz);
    m_vertices[1].set(hx , hy , -hz);
    m_vertices[2].set(-hx, hy , -hz);
    m_vertices[3].set(-hx, -hy, -hz);

    m_vertices[0].set(hx , -hy, hz);
    m_vertices[1].set(hx , hy , hz);
    m_vertices[2].set(-hx, hy , hz);
    m_vertices[3].set(-hx, -hy, hz);

    m_normals[0].set(0.0f, 0.0f, -1.0f);
    m_normals[1].set(0.0f, 1.0f, 0.0f);
    m_normals[2].set(0.0f, 0.0f, 1.0f);
    m_normals[3].set(0.0f, -1.0f, 0.0f);
    m_normals[4].set(1.0f, 0.0f, 0.0f);
    m_normals[5].set(-1.0f, 0.0f, 0.0f);

    m_xyz.set(2 * hx, 2 * hy, 2 * hz);
}


void box3d::b3CubeShape::get_bound_aabb(box3d::b3AABB *aabb, const b3TransformD &xf, int32 childIndex) const
{
    b3_NOT_USED(childIndex);

    b3Vector3d min = xf.transform(m_vertices[0]);
    b3Vector3d max = min;

    for (int32 i = 1; i < 8; ++i) {
        b3Vector3d v = xf.transform(m_vertices[i]);
        min = b3_min_coeff(min, v);
        max = b3_max_coeff(max, v);
    }

    b3Vector3d r(m_radius, m_radius, m_radius);
    aabb->m_min = min - r;
    aabb->m_max = max + r;
}


void box3d::b3CubeShape::compute_mass_properties(box3d::b3MassProperty &mass_data, double density) const
{
    mass_data.m_center = m_centroid;

    mass_data.m_volume = m_xyz.x() * m_xyz.y() * m_xyz.z();

    mass_data.m_mass = density * mass_data.m_volume;

    double x2 = m_xyz.x() * m_xyz.x();
    double y2 = m_xyz.y() * m_xyz.y();
    double z2 = m_xyz.z() * m_xyz.z();
    double ot = 1.0 / 12.0;

    double i11 = ot * mass_data.m_mass * (y2 + z2);
    double i22 = ot * mass_data.m_mass * (x2 + z2);
    double i33 = ot * mass_data.m_mass * (x2 + y2);

    mass_data.m_Inertia << i11, 0.0, 0.0,
                           0.0, i22, 0.0,
                           0.0, 0.0, i33;
}


void box3d::b3CubeShape::get_view_data(box3d::b3ViewData *view_data) const
{
    view_data->m_V.resize(8, 3);
    view_data->m_V << m_vertices[0].eigen_vector3().transpose(),
                      m_vertices[1].eigen_vector3().transpose(),
                      m_vertices[2].eigen_vector3().transpose(),
                      m_vertices[3].eigen_vector3().transpose(),
                      m_vertices[4].eigen_vector3().transpose(),
                      m_vertices[5].eigen_vector3().transpose(),
                      m_vertices[6].eigen_vector3().transpose(),
                      m_vertices[7].eigen_vector3().transpose();

    view_data->m_F.resize(12, 3);

    view_data->m_F << 0, 1, 2,
                      0, 2, 3,
                      4, 5, 6,
                      4, 6, 7,
                      0, 1, 5,
                      0, 5, 4,
                      1, 2, 6,
                      1, 6, 5,
                      2, 3, 7,
                      2, 7, 6,
                      3, 0, 4,
                      3, 4, 7;
}


box3d::b3Shape *box3d::b3CubeShape::clone() const
{
    void* mem = b3_alloc(sizeof(b3CubeShape));
    auto* clone = new (mem) b3CubeShape;
    *clone = *this;
    return clone;
}



