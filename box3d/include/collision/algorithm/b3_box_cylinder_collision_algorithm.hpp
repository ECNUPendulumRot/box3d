
#ifndef B3_BOX_CYLINDER_COLLISION_ALGORITHM_HPP
#define B3_BOX_CYLINDER_COLLISION_ALGORITHM_HPP

#include "b3_collision_algorithm.hpp"
#include "collision/b3_collision_algorithm_create_func.hpp"
#include "collision/b3_dispatcher.hpp"

class b3PersistentManifold;
class b3PenetrationDepthSolver;

class b3BoxCylinderCollisionAlgorithm : public b3CollisionAlgorithm {

    b3PenetrationDepthSolver* m_pd_solver;

    b3PersistentManifold* m_manifold_ptr;

    int m_numPerturbationIterations;
    int m_minimumPointsPerturbationThreshold;

public:

    explicit b3BoxCylinderCollisionAlgorithm(b3Dispatcher* dispatcher, b3PenetrationDepthSolver* pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold);

    void process_collision(const b3Fixture* fixtureA, const b3Fixture* fixtureB, const b3DispatcherInfo& info, b3PersistentManifold* manifold) override;

    struct CreateFunc : public b3CollisionAlgorithmCreateFunc {

        b3PenetrationDepthSolver* m_pd_solver;
        int m_numPerturbationIterations;
        int m_minimumPointsPerturbationThreshold;

        CreateFunc(b3PenetrationDepthSolver* pdSolver);

        b3CollisionAlgorithm* create_collision_algorithm(b3Dispatcher* dispatcher, const b3Fixture* fixtureA, const b3Fixture* fixtureB, int& allocate_size) override {
            allocate_size = sizeof(b3BoxCylinderCollisionAlgorithm);
            void* mem = dispatcher->allocate_collision_algorithm(allocate_size);
            return new (mem) b3BoxCylinderCollisionAlgorithm(dispatcher, m_pd_solver, m_numPerturbationIterations, m_minimumPointsPerturbationThreshold);
        }
    };
};

#endif