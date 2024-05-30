
#include "collision/b3_dispatcher.hpp"

#include "collision/b3_contact.hpp"
#include "dynamics/b3_body.hpp"
#include "collision/b3_persistent_manifold.hpp"
#include "collision/b3_fixture.hpp"
#include "geometry/b3_shape.hpp"

#include "collision/algorithm/b3_sphere_sphere_collision_algorithm.hpp"

b3Dispatcher::b3Dispatcher(b3BlockAllocator* block_allocator)
{
    m_block_allocator = block_allocator;
    // TODO: if need to 提升为类成员变量

    collision_configuration.set_block_allocator(m_block_allocator);

    // TODO:
    for (int i = 0; i < b3ShapeType::e_type_count; i++) {
        for (int j = 0; j < b3ShapeType::e_type_count; j++) {
            m_dispatch_contact_points[i][j] = collision_configuration.get_collision_algorithm_create_func(i, j);
        }
    }
}


bool b3Dispatcher::need_response(const b3Body *bodyA, const b3Body *bodyB)
{
//    bool result = bodyA->has_contact_response() && bodyB->has_contact_response();
//
//    // no response between two static/kinematic bodies
//    // Other words, bodyA or bodyB is dynamic object
//    result = result && (bodyA->is_dynamic_object() || bodyB->is_dynamic_object());
//    return result;
    return true;
}


b3PersistentManifold* b3Dispatcher::get_new_manifold(const b3Body *bodyA, const b3Body *bodyB)
{
    // TODO: compute this value by bodyA and bodyB
    real contact_breaking_threshold = 0.02;
    real contact_processing_threshold = 9.9e16;

    void* mem = m_block_allocator->allocate(sizeof(b3PersistentManifold));
    b3PersistentManifold* manifold = new (mem) b3PersistentManifold;

    b3_assert(manifold != nullptr);

    manifold->set_index(m_manifolds.size());
    m_manifolds.push_back(manifold);

    return manifold;
}


void b3Dispatcher::clear_manifold(b3PersistentManifold *manifold)
{
    manifold->clear_manifold();
}


void b3Dispatcher::release_manifold(b3PersistentManifold *manifold)
{
    clear_manifold(manifold);

    int index = manifold->get_index();

    m_manifolds[index] = m_manifolds.back();
    m_manifolds[index]->set_index(index);
    m_manifolds.pop_back();

    manifold->~b3PersistentManifold();
    m_block_allocator->free(manifold, sizeof(b3PersistentManifold));
}


bool b3Dispatcher::need_collision(const b3Body *bodyA, const b3Body *bodyB)
{
    // TODO:
    return true;
}


b3CollisionAlgorithm* b3Dispatcher::find_algorithm(
    const b3Fixture* fixtureA, const b3Fixture* fixtureB, DispatcherQueryType query_type, int& allocate_size)
{
    b3CollisionAlgorithm* algo = nullptr;
    auto s = m_dispatch_contact_points[fixtureA->get_shape()->get_type()][fixtureB->get_shape()->get_type()];
    if (s == nullptr) {
        return nullptr;
    }
    // TODO: use query_type to create function pointer
    algo = m_dispatch_contact_points[fixtureA->get_shape()->get_type()][fixtureB->get_shape()->get_type()]->create_collision_algorithm(this, fixtureA, fixtureB, allocate_size);
    return algo;
}


void b3Dispatcher::dispatch_collision_pair(b3Contact *contact, const b3DispatcherInfo& info, b3PersistentManifold* manifold)
{
    b3Fixture* fixtureA = contact->get_fixture_a();
    b3Fixture* fixtureB = contact->get_fixture_b();

    b3Body* bodyA = fixtureA->get_body();
    b3Body* bodyB = fixtureB->get_body();

    if (need_collision(bodyA, bodyB)) {

        // TODO: add a variable to contact, so reuse

        if (contact->get_algorithm() == nullptr) {
            int allocate_size;
            b3CollisionAlgorithm* algo = find_algorithm(fixtureA, fixtureB, DispatcherQueryType::B3_CONTACT_POINT_ALGORITHMS, allocate_size);

            contact->set_collision_algorithm(algo, allocate_size);
        }

        b3CollisionAlgorithm* algo = contact->get_algorithm();
        if (algo != nullptr) {
            algo->process_collision(fixtureA, fixtureB, info, manifold);
        }

    }
}


void* b3Dispatcher::allocate_collision_algorithm(int size)
{
    void* mem = m_block_allocator->allocate(size);
    b3_assert(mem != nullptr);
    return mem;
}

void b3Dispatcher::free_collision_algorithm(void* ptr, int size)
{
    m_block_allocator->free(ptr, size);
}
