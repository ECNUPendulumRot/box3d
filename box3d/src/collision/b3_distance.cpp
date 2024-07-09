
#include "collision/b3_distance.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "geometry/b3_cube_shape.hpp"
#include "geometry/b3_plane_shape.hpp"


struct b3SimplexVertex {
    b3Vec3r w_a;
    b3Vec3r w_b;
    b3Vec3r w;
    real a;
    int32 index_a;
    int32 index_b;
};


struct b3Simplex {

    void read_cache(	const b3SimplexCache* cache,
                       const b3DistanceProxy* proxyA, const b3Transr& transform_a,
                       const b3DistanceProxy* proxyB, const b3Transr& transform_b)
    {
        b3_assert(cache->count <= 3);

        // Copy data from cache.
        m_count = cache->count;
        b3SimplexVertex* vertices = &m_v1;
        for (int32 i = 0; i < m_count; ++i)
        {
            b3SimplexVertex* v = vertices + i;
            v->index_a = cache->index_a[i];
            v->index_b = cache->index_b[i];
            b3Vec3r w_a_local = proxyA->get_vertex(v->index_a);
            b3Vec3r w_b_local = proxyB->get_vertex(v->index_b);
            v->w_a = transform_a.transform(w_a_local);
            v->w_b = transform_b.transform(w_b_local);
            v->w = v->w_b - v->w_a;
            v->a = 0.0f;
        }

        // Compute the new simplex metric, if it is substantially different than
        // old metric then flush the simplex.
        if (m_count > 1)
        {
            float metric1 = cache->metric;
            float metric2 = get_metric();
            if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < b3_real_epsilon)
            {
                // Reset the simplex.
                m_count = 0;
            }
        }

        // If the cache is empty or invalid ...
        if (m_count == 0)
        {
            b3SimplexVertex* v = vertices + 0;
            v->index_a = 0;
            v->index_b = 0;
            b3Vec3r w_a_local = proxyA->get_vertex(0);
            b3Vec3r w_b_local = proxyB->get_vertex(0);
            v->w_a = transform_a.transform(w_a_local);
            v->w_b = transform_b.transform(w_b_local);
            v->w = v->w_b - v->w_a;
            v->a = 1.0;
            m_count = 1;
        }
    }

    real get_metric() const{
        switch (m_count)
        {
            case 0:
                b3_assert(false);
                return 0.0f;

            case 1:
                return 0.0f;

            case 2:
                //return b3_distance(m_v1.w, m_v2.w, m_v3.w);

            case 3:
                //return b2Cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);

            default:
                b3_assert(false);
                return 0.0f;
        }
    }

    void Solve2();
    void Solve3();

    b3SimplexVertex m_v1, m_v2, m_v3, m_v4;
    int32 m_count;
};


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
    const b3DistanceProxy* proxy_a = &input->proxy_a;
    const b3DistanceProxy* proxy_b = &input->proxy_b;

    b3Transr transform_a = input->transform_a;
    b3Transr transform_b = input->transform_b;

    b3Simplex simplex;
    simplex.read_cache(cache, proxy_a, transform_a, proxy_b, transform_b);

    // Get simplex vertices as an array.
    b3SimplexVertex* vertices = &simplex.m_v1;
    const int32 k_maxIters = 20;
    ///to be Continue
}

bool b3_shape_cast(){

}

