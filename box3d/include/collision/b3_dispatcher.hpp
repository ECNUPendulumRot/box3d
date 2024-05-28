
#ifndef B3_DISPATCHER_INFO_HPP
#define B3_DISPATCHER_INFO_HPP

#include "common/b3_common.hpp"
#include "geometry/b3_shape.hpp"
#include <vector>
#include "collision/b3_collision_config.hpp"

class b3BlockAllocator;
class b3Draw;
class b3CollisionAlgorithm;
class b3PersistentManifold;
class b3Body;
class b3Contact;
class b3Fixture;
class b3CollisionConfiguration;
struct b3CollisionAlgorithmCreateFunc;

struct b3DispatcherInfo {

    b3DispatcherInfo() : m_time_step(0), m_step_count(0), m_debug_draw(nullptr) {}

    real m_time_step;
    int m_step_count;
    b3Draw* m_debug_draw;
};

enum class DispatcherQueryType {
    B3_CONTACT_POINT_ALGORITHMS,
    b3_CLOSEST_POINT_ALGORITHMS
};

///The btDispatcher interface class can be used in combination with broadphase to dispatch calculations for overlapping pairs.
///For example for pairwise collision detection, calculating contact points stored in btPersistentManifold or user callbacks (game logic).
class b3Dispatcher {

protected:

    std::vector<b3PersistentManifold*> m_manifolds;

    b3BlockAllocator* m_block_allocator;

    b3CollisionConfiguration collision_configuration;

    b3CollisionAlgorithmCreateFunc* m_dispatch_contact_points[b3ShapeType::e_type_count][b3ShapeType::e_type_count];

public:

    b3Dispatcher(b3BlockAllocator* block_allocator);

    int get_manifolds_count() const {
        return m_manifolds.size();
    }

    b3PersistentManifold* get_manifold_by_index(int index) {
        b3_assert(index >= 0 && index < m_manifolds.size());
        return m_manifolds[index];
    }

    ~b3Dispatcher();

    b3PersistentManifold* get_new_manifold(const b3Body* bodyA, const b3Body* bodyB);

    void release_manifold(b3PersistentManifold* manifold);

    void clear_manifold(b3PersistentManifold* manifold);

    b3CollisionAlgorithm* find_algorithm(const b3Fixture* fixtureA, const b3Fixture* fixtureB, DispatcherQueryType query_type, int& allocate_size);

    bool need_collision(const b3Body* bodyA, const b3Body* bodyB);

    bool need_response(const b3Body* bodyA, const b3Body* bodyB);

    void dispatch_collision_pair(b3Contact* contact, const b3DispatcherInfo& info, b3PersistentManifold* manifold);

    void* allocate_collision_algorithm(int size);

    void free_collision_algorithm(void* ptr, int size);
};

#endif