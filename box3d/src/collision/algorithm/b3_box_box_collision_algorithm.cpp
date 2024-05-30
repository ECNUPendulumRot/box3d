
#include "collision/algorithm/b3_box_box_collision_algorithm.hpp"

#include "collision/b3_fixture.hpp"
#include "geometry/b3_cube_shape.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_persistent_manifold.hpp"
#include "collision/b3_manifold_result.hpp"
#include "collision/algorithm/b3_box_box_detector.hpp"

b3BoxBoxCollisionAlgorithm::b3BoxBoxCollisionAlgorithm(b3Dispatcher *dispatcher) : b3CollisionAlgorithm(dispatcher)
{
}


void b3BoxBoxCollisionAlgorithm::process_collision(
    const b3Fixture *fixtureA, const b3Fixture *fixtureB,
    const b3DispatcherInfo &info, b3PersistentManifold *manifold)
{
    b3Body* bodyA = fixtureA->get_body();
    b3Body* bodyB = fixtureB->get_body();

    b3ManifoldResult result(bodyA, bodyB, manifold);

    b3BoxBoxDetector detector(fixtureA, fixtureB);
    detector.get_closest_points(result);

    result.refresh_contact_points();
}

