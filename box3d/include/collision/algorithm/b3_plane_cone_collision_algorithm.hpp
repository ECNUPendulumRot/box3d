
#ifndef B3_PLANE_CONE_COLLISION_ALGORITHM_H
#define B3_PLANE_CONE_COLLISION_ALGORITHM_H

#include "b3_collision_algorithm.hpp"
#include "collision/b3_collision_algorithm_create_func.hpp"
#include "collision/b3_dispatcher.hpp"

class b3PersistentManifold;

class b3PlaneConeCollisionAlgorithm : public b3CollisionAlgorithm {

public:

    explicit b3PlaneConeCollisionAlgorithm(b3Dispatcher* dispatcher);

    void process_collision(const b3Fixture* fixtureA, const b3Fixture* fixtureB, const b3DispatcherInfo& info, b3PersistentManifold* manifold) override;

    struct CreateFunc : public b3CollisionAlgorithmCreateFunc {
        b3CollisionAlgorithm* create_collision_algorithm(b3Dispatcher* dispatcher, const b3Fixture* fixtureA, const b3Fixture* fixtureB, int& allocate_size) override {
            allocate_size = sizeof(b3PlaneConeCollisionAlgorithm);
            void* mem = dispatcher->allocate_collision_algorithm(allocate_size);
            return new (mem) b3PlaneConeCollisionAlgorithm(dispatcher);
        }
    };
};


#endif