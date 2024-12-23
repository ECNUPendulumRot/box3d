
#ifndef B3_COLLISION_CONFIG_HPP
#define B3_COLLISION_CONFIG_HPP

#include "b3_collision_algorithm_create_func.hpp"

class b3BlockAllocator;

class b3PenetrationDepthSolver;


class b3CollisionConfiguration {

private:

    b3PenetrationDepthSolver* m_pd_solver;

    b3BlockAllocator* m_block_allocator;

    b3CollisionAlgorithmCreateFunc* m_sphere_sphere_cf;

    b3CollisionAlgorithmCreateFunc* m_box_sphere_cf;

    b3CollisionAlgorithmCreateFunc* m_box_box_cf;

    b3CollisionAlgorithmCreateFunc* m_plane_sphere_cf;

    b3CollisionAlgorithmCreateFunc* m_plane_box_cf;

    b3CollisionAlgorithmCreateFunc* m_plane_cone_cf;

    b3CollisionAlgorithmCreateFunc* m_box_cone_cf;

    b3CollisionAlgorithmCreateFunc* m_box_cylinder_cf;

    void init();

public:

    void set_block_allocator(b3BlockAllocator* block_allocator) {
        m_block_allocator = block_allocator;
        init();
    }

    ~b3CollisionConfiguration();

    b3CollisionAlgorithmCreateFunc* get_collision_algorithm_create_func(int shape_typeA, int shape_typeB);
};


#endif