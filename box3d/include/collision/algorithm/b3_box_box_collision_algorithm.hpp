
#ifndef B3_BOX_BOX_COLLISION_ALGORITHM_HPP
#define B3_BOX_BOX_COLLISION_ALGORITHM_HPP

#include "b3_collision_algorithm.hpp"
#include "collision/b3_collision_algorithm_create_func.hpp"
#include "collision/b3_dispatcher.hpp"

class b3PersistentManifold;

class b3BoxBoxCollisionAlgorithm : public b3CollisionAlgorithm {

    b3PersistentManifold* m_manifold;

public:

    b3BoxBoxCollisionAlgorithm(b3Dispatcher* dispatcher);

    ~b3BoxBoxCollisionAlgorithm();

    void process_collision(const b3Fixture* fixtureA, const b3Fixture* fixtureB, const b3DispatcherInfo& info, b3PersistentManifold* manifold) override;

    struct CreateFunc : public b3CollisionAlgorithmCreateFunc {
        b3CollisionAlgorithm* create_collision_algorithm(b3Dispatcher* dispatcher, const b3Fixture* fixtureA, const b3Fixture* fixtureB, int& allocate_size) override {
            allocate_size = sizeof(b3BoxBoxCollisionAlgorithm);
            void* mem = dispatcher->allocate_collision_algorithm(allocate_size);
            return new (mem) b3BoxBoxCollisionAlgorithm(dispatcher);
        }
    };
};


#endif