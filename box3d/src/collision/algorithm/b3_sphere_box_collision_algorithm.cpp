
#include "collision/algorithm/b3_sphere_box_collision_algorithm.hpp"

#include "collision/b3_persistent_manifold.hpp"
#include "collision/b3_manifold_result.hpp"
#include "collision/b3_fixture.hpp"
#include "collision/b3_manifold_result.hpp"
#include "dynamics/b3_body.hpp"

b3SphereBoxCollisionAlgorithm::b3SphereBoxCollisionAlgorithm(b3Dispatcher* dispatcher) : b3CollisionAlgorithm(dispatcher)
{
    m_manifold = nullptr;
}


b3SphereBoxCollisionAlgorithm::~b3SphereBoxCollisionAlgorithm()
{
    if (m_manifold) {
        m_dispatcher->release_manifold(m_manifold);
    }
}


void b3SphereBoxCollisionAlgorithm::process_collision(
    const b3Fixture *fixtureA, const b3Fixture *fixtureB,
    const b3DispatcherInfo &info, b3PersistentManifold *manifold)
{
    if (m_manifold == nullptr) {
        m_manifold = manifold;
    }

    // bodyA is the sphere, bodyB is the box
    b3Body* bodyA = fixtureA->get_body();
    b3Body* bodyB = fixtureB->get_body();

    b3ManifoldResult result(bodyA, bodyB);
    result.set_persistent_manifold(manifold);

    b3Vec3r sphere_center = bodyA->get_position();
    const b3Shape* shapeA = fixtureA->get_shape();

    real radius = shapeA->get_radius();
    real max_contact_distance = m_manifold->get_contact_breaking_threshold();

    // TODO:
    // if overlapped, add contact point

    if (manifold->get_contact_point_count() > 0) {
        result.refresh_contact_points();
    }

}
