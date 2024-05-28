
#include "collision/algorithm/b3_sphere_sphere_collision_algorithm.hpp"

#include "collision/b3_fixture.hpp"
#include "geometry/b3_sphere_shape.hpp"
#include "dynamics/b3_body.hpp"

#include "collision/b3_persistent_manifold.hpp"
#include "collision/b3_manifold_result.hpp"

b3SphereSphereCollisionAlgorithm::b3SphereSphereCollisionAlgorithm(b3Dispatcher *dispatcher) : b3CollisionAlgorithm(dispatcher)
{
    m_manifold = nullptr;
}


b3SphereSphereCollisionAlgorithm::~b3SphereSphereCollisionAlgorithm()
{
    if (m_manifold) {
        m_dispatcher->release_manifold(m_manifold);
    }
}


void b3SphereSphereCollisionAlgorithm::process_collision(
    const b3Fixture *fixtureA, const b3Fixture *fixtureB,
    const b3DispatcherInfo &info, b3PersistentManifold* manifold)
{
    if (m_manifold == nullptr) {
        m_manifold = manifold;
    }

    b3SphereShape* shapeA = (b3SphereShape*)fixtureA->get_shape();
    b3SphereShape* shapeB = (b3SphereShape*)fixtureB->get_shape();

    b3Body* bodyA = fixtureA->get_body();
    b3Body* bodyB = fixtureB->get_body();

    b3Vec3r diff = bodyB->get_position() - bodyA->get_position();
    real length = diff.length();
    real radiusA = shapeA->get_radius();
    real radiusB = shapeB->get_radius();

    if (length > radiusA + radiusB) {
        return;
    }

    // TODO:
    m_manifold->clear_manifold();

    real dist = length - radiusA - radiusB;

    b3Vec3r normal(1, 0, 0);
    if (length > b3_real_epsilon) {
        normal = diff / length;
    }

    // point on A (world space)
    b3Vec3r pointA = bodyA->get_position() + normal * radiusA;
    // point on B (world space)
    b3Vec3r pointB = bodyB->get_position() - normal * radiusB;

    /// report a contact. internally this will be kept persistent, and contact reduction is done
    b3ManifoldResult result(bodyA, bodyB);
    result.set_persistent_manifold(m_manifold);
    result.add_contact_point(normal, pointA, dist);
}

