
#ifndef B3_COLLISION_ALGORITHM_CREATE_FUNC_HPP
#define B3_COLLISION_ALGORITHM_CREATE_FUNC_HPP

class b3CollisionAlgorithm;
class b3Dispatcher;
class b3Fixture;

struct b3CollisionAlgorithmCreateFunc {

    bool m_swapped;

    b3CollisionAlgorithmCreateFunc() : m_swapped(false) {}

    virtual ~b3CollisionAlgorithmCreateFunc() {}

    virtual b3CollisionAlgorithm* create_collision_algorithm(b3Dispatcher* dispatcher, const b3Fixture* fixtureA, const b3Fixture* fixtureB, int& allocate_size) {
        (void)dispatcher;
        (void)fixtureA;
        (void)fixtureB;
        return nullptr;
    }
};

#endif