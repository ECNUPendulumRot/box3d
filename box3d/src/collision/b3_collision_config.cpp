
#include "collision/b3_collision_config.hpp"

#include "common/b3_block_allocator.hpp"
#include "collision/algorithm/b3_sphere_sphere_collision_algorithm.hpp"
#include "geometry/b3_shape.hpp"

void b3CollisionConfiguration::init()
{
    void* mem = m_block_allocator->allocate(sizeof(b3SphereSphereCollisionAlgorithm::CreateFunc));
    m_sphere_sphere_cf = new (mem) b3SphereSphereCollisionAlgorithm::CreateFunc();
}


b3CollisionConfiguration::~b3CollisionConfiguration()
{

    m_sphere_sphere_cf->~b3CollisionAlgorithmCreateFunc();
    m_block_allocator->free(m_sphere_sphere_cf, sizeof(b3SphereSphereCollisionAlgorithm::CreateFunc));

    m_block_allocator = nullptr;
}

b3CollisionAlgorithmCreateFunc* b3CollisionConfiguration::get_collision_algorithm_create_func(int shape_typeA, int shape_typeB)
{
    if (shape_typeA == b3ShapeType::e_sphere && shape_typeB == b3ShapeType::e_sphere) {
        return m_sphere_sphere_cf;
    }
    return nullptr;
}