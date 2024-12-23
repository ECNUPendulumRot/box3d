
#include "geometry/b3_shape.hpp"
#include "common/b3_block_allocator.hpp"

#include "geometry/b3_cube_shape.hpp"
#include "geometry/b3_cylinder_shape.hpp"

b3Shape::~b3Shape()
{
    m_block_allocator->free(m_view_data.m_V, 3 * sizeof(double) * m_view_data.m_vertex_count);
    m_block_allocator->free(m_view_data.m_F, 3 * sizeof(int) * m_view_data.m_face_count);
}


b3Vec3r b3Shape::local_get_support_vertex(const b3Vec3r &local_dir) const {
    switch (m_type) {
        case b3ShapeType::e_cube: {
            b3CubeShape* shape = (b3CubeShape *) this;
            const b3Vec3r half_extents = shape->m_h_xyz_no_margin;
            return {
                local_dir.x >= 0 ? half_extents.x : -half_extents.x,
                local_dir.y >= 0 ? half_extents.y : -half_extents.y,
                local_dir.z >= 0 ? half_extents.z : -half_extents.z
            };
        }
        case b3ShapeType::e_cylinder: {
            b3CylinderShape* shape = (b3CylinderShape*)this;
            real radius = shape->get_radius();
            real half_height = shape->get_height() * 0.5f;

            b3Vec3r res;
            real s = b3_sqrt(local_dir.x * local_dir.x + local_dir.y * local_dir.y);
            if (s != 0) {
                real d = radius / s;
                return {
                    local_dir.x * d,
                    local_dir.y * d,
                    local_dir.z < 0.0 ? -half_height : half_height
                };
            } else {
                return { radius, 0, local_dir.z < 0.0 ? -half_height : half_height };
            }
        }
        default:
            return this->local_get_support_vertex(local_dir);
    }

    return b3Vec3r::zero();
}