
#include "collision/gjk/b3_penetration_depth_solver.hpp"

#include "dynamics/b3_transform.hpp"
#include "math/b3_vec3.hpp"

#include "collision/gjk/b3_gjk_epa_solver2.hpp"

bool b3GjkEpaPenetrationDepthSolver::calcPenDepth(b3VoronoiSimplexSolver &simplex_solver, const b3Shape *shapeA, const b3Shape *shapeB,
    const b3Transformr &transformA, const b3Transformr &transformB, b3Vec3r &v, b3Vec3r &w_witness_onA, b3Vec3r &w_witness_onB) {

    b3Vec3r guessVectors[] = {
        b3Vec3r(transformB.position() - transformA.position()).safe_normalized(),
        b3Vec3r(transformA.position() - transformB.position()).safe_normalized(),
        b3Vec3r(0, 0, 1),
        b3Vec3r(0, 1, 0),
        b3Vec3r(1, 0, 0),
        b3Vec3r(1, 1, 0),
        b3Vec3r(1, 1, 1),
        b3Vec3r(0, 1, 1),
        b3Vec3r(1, 0, 1),
    };

    int numVectors = sizeof(guessVectors) / sizeof(b3Vec3r);

    for (int i = 0; i < numVectors; i++)
    {
        simplex_solver.reset();
        b3Vec3r guessVector = guessVectors[i];

        b3GjkEpaSolver2::sResults results;

        if (b3GjkEpaSolver2::Penetration(shapeA, transformA,
                                         shapeB, transformB,
                                         guessVector, results))

        {
            w_witness_onA = results.witnesses[0];
            w_witness_onB = results.witnesses[1];
            v = results.normal;
            return true;
        }
        else
        {
            if (b3GjkEpaSolver2::Distance(shapeA, transformA, shapeB, transformB, guessVector, results))
            {
                w_witness_onA = results.witnesses[0];
                w_witness_onB = results.witnesses[1];
                v = results.normal;
                return false;
            }
        }
    }

    //failed to find a distance/penetration
    w_witness_onA.set(0, 0, 0);
    w_witness_onB.set(0, 0, 0);
    v.set(0, 0, 0);
    return false;
}