
#ifndef B3_PENETRATION_DEPTH_SOLVER_HPP
#define B3_PENETRATION_DEPTH_SOLVER_HPP

#include "collision/gjk/b3_voronoi_simplex_solver.hpp"
#include "dynamics/b3_transform.hpp"

class b3Shape;

class b3PenetrationDepthSolver {
public:
    virtual ~b3PenetrationDepthSolver() = default;

    virtual bool calcPenDepth(b3VoronoiSimplexSolver& simplex_solver,
                      const b3Shape* shapeA, const b3Shape* shapeB,
                      const b3Transformr& transformA, const b3Transformr& transformB,
                      b3Vec3r& v, b3Vec3r& w_witness_onA, b3Vec3r& w_witness_onB) { return true;}
};

class b3GjkEpaPenetrationDepthSolver : public b3PenetrationDepthSolver {
public:
    b3GjkEpaPenetrationDepthSolver() = default;

    virtual bool calcPenDepth(b3VoronoiSimplexSolver& simplex_solver,
                      const b3Shape* shapeA, const b3Shape* shapeB,
                      const b3Transformr& transformA, const b3Transformr& transformB,
                      b3Vec3r& v, b3Vec3r& w_witness_onA, b3Vec3r& w_witness_onB) override;
};

#endif