
#ifndef B3_GJK_PAIR_DETECTOR_HPP
#define B3_GJK_PAIR_DETECTOR_HPP

#include "collision/gjk/b3_voronoi_simplex_solver.hpp"
#include "dynamics/b3_transform.hpp"

class b3Shape;
class b3PenetrationDepthSolver;
class b3ManifoldResult;

class b3GjkPairDetector {

    b3Vec3r m_cached_separating_axis;
    b3PenetrationDepthSolver* m_penetration_depth_solver;
    b3VoronoiSimplexSolver* m_simplex_solver;
    const b3Shape* m_minkowskiA;
    const b3Shape* m_minkowskiB;
    real m_marginA;
    real m_marginB;

    real m_cached_separating_distance;

public:

    struct ClosestPointInput {

        ClosestPointInput() : m_maximum_distance_squared(b3_real_max) {}

        b3Transformr m_transformA;
        b3Transformr m_transformB;
        real m_maximum_distance_squared;
    };

    int m_cur_iter;
    int m_degenerate_simplex;

    b3GjkPairDetector(const b3Shape* shapeA, const b3Shape* shapeB, b3VoronoiSimplexSolver* simplex_solver, b3PenetrationDepthSolver* penetration_depth_solver);

    void get_closest_points(const ClosestPointInput& input, b3ManifoldResult& result);

    virtual ~b3GjkPairDetector() = default;
};

#endif