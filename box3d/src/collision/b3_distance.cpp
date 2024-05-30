
#include "collision/b3_distance.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_cube_shape.hpp"
#include "geometry/b3_plane_shape.hpp"


void b3DistanceProxy::set(const b3Shape *shape, int32 index)
{
    switch (shape->get_type()) {
        case b3ShapeType::e_sphere: {
            const b3SphereShape *sphere = (const b3SphereShape *) shape;
            m_vertices = &sphere->m_centroid;
            m_count = 1;
            m_radius = sphere->m_radius;
            break;
        }
        case b3ShapeType::e_cube: {
            const b3CubeShape *cube = (const b3CubeShape *) shape;
            m_vertices = cube->m_vertices;
            m_count = 8;
            m_radius = cube->m_radius;
            break;
        }
        case b3ShapeType::e_plane: {
            const b3PlaneShape *plane = (const b3PlaneShape *) shape;
            m_vertices = plane->m_vertices;
            m_count = 4;
            m_radius = plane->m_radius;
            break;
        }
        default:
            b3_assert(false);
    }
}


void b3_distance(b3DistanceOutput *output, b3SimplexCache *cache, const b3DistanceInput *input)
{

}
