
#ifndef B3_COLLISION_ALGORITHM_HPP
#define B3_COLLISION_ALGORITHM_HPP

class b3Dispatcher;
class b3Fixture;
class b3DispatcherInfo;
class b3PersistentManifold;

class b3CollisionAlgorithm {

protected:
    b3Dispatcher* m_dispatcher;

public:

    b3CollisionAlgorithm(b3Dispatcher* dispatcher) {
        m_dispatcher = dispatcher;
    }

    virtual ~b3CollisionAlgorithm() = default;

    virtual void process_collision(const b3Fixture* fixtureA, const b3Fixture* fixtureB,
                                   const b3DispatcherInfo& info, b3PersistentManifold* manifold) = 0;

};

#endif