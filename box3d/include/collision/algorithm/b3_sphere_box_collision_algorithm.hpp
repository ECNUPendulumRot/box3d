
#ifndef B3_SPHERE_BOX_COLLISION_ALGORITHM_H
#define B3_SPHERE_BOX_COLLISION_ALGORITHM_H

#include "b3_collision_algorithm.hpp"
#include "collision/b3_collision_algorithm_create_func.hpp"
#include "collision/b3_dispatcher.hpp"

class b3PersistentManifold;

class b3CubeShape;

class b3SphereBoxCollisionAlgorithm : public b3CollisionAlgorithm {

    b3PersistentManifold* m_manifold;

public:

    b3SphereBoxCollisionAlgorithm(b3Dispatcher* dispatcher);

    bool get_sphere_distance(const b3CubeShape* shapeA, b3Transformr& xfA, b3Vec3r& point_on_box, b3Vec3r& normal, real& penetration, const b3Vec3r& sphere_center, real sphere_radius);

    real find_normal(const b3Vec3r& box_half_xyz, const b3Vec3r& local_sphere_center, b3Vec3r& normal, b3Vec3r& closest_point);

    virtual ~b3SphereBoxCollisionAlgorithm();

    void process_collision(const b3Fixture* fixtureA, const b3Fixture* fixtureB, const b3DispatcherInfo& info, b3PersistentManifold* manifold) override;

    struct CreateFunc : public b3CollisionAlgorithmCreateFunc {
        b3CollisionAlgorithm* create_collision_algorithm(b3Dispatcher* dispatcher, const b3Fixture* fixtureA, const b3Fixture* fixtureB, int& allocate_size) override {
            allocate_size = sizeof(b3SphereBoxCollisionAlgorithm);
            void* mem = dispatcher->allocate_collision_algorithm(allocate_size);
            return new (mem) b3SphereBoxCollisionAlgorithm(dispatcher);
        }
    };

};


#endif