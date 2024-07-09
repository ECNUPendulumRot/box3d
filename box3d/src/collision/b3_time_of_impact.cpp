
#include "collision/b3_time_of_impact.hpp"


int32 b3_toi_calls, b2_toi_iters, b2_toi_max_iters;

void b3_time_of_impact(b3TOIOutput *output, const b3TOIInput *input)
{
    output->state = b3TOIOutput::e_unknown;
    output->t = input->t_max;

    const b3DistanceProxy &proxy_a = input->proxy_a;
    const b3DistanceProxy &proxy_b = input->proxy_b;

    b3Sweep sweep_a = input->sweep_a;
    b3Sweep sweep_b = input->sweep_b;

    real t_max = input->t_max;
    real total_radius = proxy_a.m_radius + proxy_b.m_radius;
    real target = b3_max(b3_linear_slop, total_radius - 3.0 * b3_linear_slop);
    real tolerance = 0.25 * b3_linear_slop;

    b3_assert(target > tolerance);

    real t1 = 0.0f;
    const int32 k_max_iterations = 20;
    int32 iter = 0;


    b3SimplexCache cache;
    cache.count = 0;
    b3DistanceInput distance_input;
    distance_input.proxy_a = input->proxy_a;
    distance_input.proxy_b = input->proxy_b;
    distance_input.use_radii = false;

    for (;;) {
        b3Transr xf_a, xf_b;
        sweep_a.get_transform(xf_a, t1);
        sweep_b.get_transform(xf_b, t1);

        distance_input.transform_a = xf_a;
        distance_input.transform_b = xf_b;
        b3DistanceOutput distance_output;
        b3_distance(&distance_output, &cache, &distance_input);
    }
}

struct b3_separation_function{
    enum Type{
        e_points,
        e_faceA,
        e_faceB,
        e_edges
    };

    real initialize(const b3SimplexCache* cache,
                    const b3DistanceProxy* proxyA, const b3Sweep& sweepA,
                    const b3DistanceProxy* proxyB, const b3Sweep& sweepB,
                    real t1){
        m_proxyA = proxyA;
        m_proxyB = proxyB;
        int32 count = cache->count;
        b3_assert(0 < count && count < 3);

        m_sweepA = sweepA;
        m_sweepB = sweepB;

        b3Transr xfA, xfB;
        m_sweepA.get_transform(xfA, t1);
        m_sweepB.get_transform(xfB, t1);


    }

    const b3DistanceProxy* m_proxyA;
    const b3DistanceProxy* m_proxyB;
    b3Sweep m_sweepA, m_sweepB;
    Type m_type;
    b3Vec3r m_localPoint;
    b3Vec3r m_axis;
};