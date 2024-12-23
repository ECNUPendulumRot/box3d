
#include "collision/algorithm/b3_box_cylinder_collision_algorithm.hpp"

#include "collision/b3_fixture.hpp"
#include "geometry/b3_cylinder_shape.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_persistent_manifold.hpp"
#include "collision/b3_manifold_result.hpp"

#include "collision/gjk/b3_gjk_pair_detector.hpp"

b3BoxCylinderCollisionAlgorithm::b3BoxCylinderCollisionAlgorithm(b3Dispatcher* dispatcher, b3PenetrationDepthSolver* pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold)
    : b3CollisionAlgorithm(dispatcher), m_pd_solver(pdSolver), m_numPerturbationIterations(numPerturbationIterations),
      m_minimumPointsPerturbationThreshold(minimumPointsPerturbationThreshold)
{
}

b3BoxCylinderCollisionAlgorithm::CreateFunc::CreateFunc(b3PenetrationDepthSolver *pdSolver) {
    m_numPerturbationIterations = 0;
    m_minimumPointsPerturbationThreshold = 3;
    m_pd_solver = pdSolver;
}

void b3BoxCylinderCollisionAlgorithm::process_collision(
    const b3Fixture *fixtureA, const b3Fixture *fixtureB,
    const b3DispatcherInfo &info, b3PersistentManifold* manifold)
{
    // fixtureA is box and fixtureB is cylinder
    const b3Shape* shapeA = fixtureA->get_shape();
    const b3Shape* shapeB = fixtureB->get_shape();

    const b3Body* bodyA = fixtureA->get_body();
    const b3Body* bodyB = fixtureB->get_body();

    b3GjkPairDetector::ClosestPointInput input;
    b3VoronoiSimplexSolver simplex_solver;
    b3GjkPairDetector gjk_pair_detector(shapeA, shapeB, &simplex_solver, m_pd_solver);

    input.m_maximum_distance_squared = shapeA->get_margin() + shapeB->get_margin() + manifold->get_contact_breaking_threshold();
    input.m_maximum_distance_squared *= input.m_maximum_distance_squared;

    input.m_maximum_distance_squared = 0.025;

//    input.m_transformA = b3Transformr(bodyA->get_position(), bodyA->get_quaternion());
//    input.m_transformB = b3Transformr(bodyB->get_position(), bodyB->get_quaternion());
    input.m_transformA = bodyA->get_world_transform();
    input.m_transformB = bodyB->get_world_transform();

    b3ManifoldResult result(bodyA, bodyB, manifold);
    gjk_pair_detector.get_closest_points(input, result);
    result.refresh_contact_points();
}

