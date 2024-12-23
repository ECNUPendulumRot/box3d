
#include "collision/b3_collision_config.hpp"

#include "common/b3_block_allocator.hpp"

#include "geometry/b3_shape.hpp"

#include "collision/algorithm/b3_sphere_sphere_collision_algorithm.hpp"
#include "collision/algorithm/b3_sphere_box_collision_algorithm.hpp"
#include "collision/algorithm/b3_box_box_collision_algorithm.hpp"
#include "collision/algorithm/b3_plane_sphere_collision_algorithm.hpp"
#include "collision/algorithm/b3_plane_box_collision_algorithm.hpp"
#include "collision/algorithm/b3_plane_cone_collision_algorithm.hpp"
#include "collision/algorithm/b3_box_cone_collision_algorithm.hpp"

#include "collision/gjk/b3_penetration_depth_solver.hpp"
#include "collision/algorithm/b3_box_cylinder_collision_algorithm.hpp"


void b3CollisionConfiguration::init()
{
    void* mem = m_block_allocator->allocate(sizeof(b3SphereSphereCollisionAlgorithm::CreateFunc));
    m_sphere_sphere_cf = new (mem) b3SphereSphereCollisionAlgorithm::CreateFunc();

    mem = m_block_allocator->allocate(sizeof(b3SphereBoxCollisionAlgorithm::CreateFunc));
    m_box_sphere_cf = new (mem) b3SphereBoxCollisionAlgorithm::CreateFunc();

    mem = m_block_allocator->allocate(sizeof(b3BoxBoxCollisionAlgorithm::CreateFunc));
    m_box_box_cf = new (mem) b3BoxBoxCollisionAlgorithm::CreateFunc();

    mem = m_block_allocator->allocate(sizeof(b3PlaneSphereCollisionAlgorithm::CreateFunc));
    m_plane_sphere_cf = new (mem) b3PlaneSphereCollisionAlgorithm::CreateFunc();

    mem = m_block_allocator->allocate(sizeof(b3PlaneBoxCollisionAlgorithm::CreateFunc));
    m_plane_box_cf = new (mem) b3PlaneBoxCollisionAlgorithm::CreateFunc();

    mem = m_block_allocator->allocate(sizeof(b3PlaneConeCollisionAlgorithm::CreateFunc));
    m_plane_cone_cf = new (mem) b3PlaneConeCollisionAlgorithm::CreateFunc();

    mem = m_block_allocator->allocate(sizeof(b3GjkEpaPenetrationDepthSolver));
    m_pd_solver = new (mem) b3GjkEpaPenetrationDepthSolver;
    mem = m_block_allocator->allocate(sizeof(b3BoxConeCollisionAlgorithm::CreateFunc));
    m_box_cone_cf = new (mem) b3BoxConeCollisionAlgorithm::CreateFunc(m_pd_solver);

    mem = m_block_allocator->allocate(sizeof(b3BoxCylinderCollisionAlgorithm::CreateFunc));
    m_box_cylinder_cf = new (mem) b3BoxCylinderCollisionAlgorithm::CreateFunc(m_pd_solver);
}


b3CollisionConfiguration::~b3CollisionConfiguration()
{
    m_sphere_sphere_cf->~b3CollisionAlgorithmCreateFunc();
    m_block_allocator->free(m_sphere_sphere_cf, sizeof(b3SphereSphereCollisionAlgorithm::CreateFunc));

    m_box_sphere_cf->~b3CollisionAlgorithmCreateFunc();
    m_block_allocator->free(m_box_sphere_cf, sizeof(b3SphereBoxCollisionAlgorithm::CreateFunc));

    m_box_box_cf->~b3CollisionAlgorithmCreateFunc();
    m_block_allocator->free(m_box_box_cf, sizeof(b3BoxBoxCollisionAlgorithm::CreateFunc));

    m_plane_sphere_cf->~b3CollisionAlgorithmCreateFunc();
    m_block_allocator->free(m_plane_sphere_cf, sizeof(b3PlaneSphereCollisionAlgorithm::CreateFunc));

    m_plane_box_cf->~b3CollisionAlgorithmCreateFunc();
    m_block_allocator->free(m_plane_box_cf, sizeof(b3PlaneBoxCollisionAlgorithm::CreateFunc));

    m_plane_cone_cf->~b3CollisionAlgorithmCreateFunc();
    m_block_allocator->free(m_plane_cone_cf, sizeof(b3PlaneConeCollisionAlgorithm::CreateFunc));

    m_box_cone_cf->~b3CollisionAlgorithmCreateFunc();
    m_block_allocator->free(m_box_cone_cf, sizeof(b3BoxConeCollisionAlgorithm::CreateFunc));

    m_box_cylinder_cf->~b3CollisionAlgorithmCreateFunc();
    m_block_allocator->free(m_box_cylinder_cf, sizeof(b3BoxCylinderCollisionAlgorithm::CreateFunc));

    m_block_allocator = nullptr;
}


b3CollisionAlgorithmCreateFunc* b3CollisionConfiguration::get_collision_algorithm_create_func(int shape_typeA, int shape_typeB)
{
    if (shape_typeA == b3ShapeType::e_sphere && shape_typeB == b3ShapeType::e_sphere) {
        return m_sphere_sphere_cf;
    }

    // do not exist this situation: shape_typeA == b3ShapeType::e_sphere && shape_typeB == b3ShapeType::e_cube
    if (shape_typeA == b3ShapeType::e_cube && shape_typeB == b3ShapeType::e_sphere) {
        return m_box_sphere_cf;
    }

    if (shape_typeA == b3ShapeType::e_cube && shape_typeB == b3ShapeType::e_cube) {
        return m_box_box_cf;
    }

    if (shape_typeA == b3ShapeType::e_plane && shape_typeB == b3ShapeType::e_sphere) {
        return m_plane_sphere_cf;
    }

    if (shape_typeA == b3ShapeType::e_plane && shape_typeB == b3ShapeType::e_cube) {
        return m_plane_box_cf;
    }

    if (shape_typeA == b3ShapeType::e_plane && shape_typeB == b3ShapeType::e_cone) {
        return m_plane_cone_cf;
    }

    if (shape_typeA == b3ShapeType::e_cube && shape_typeB == b3ShapeType::e_cone) {
        return m_box_cone_cf;
    }

    if (shape_typeA == b3ShapeType::e_cube && shape_typeB == b3ShapeType::e_cylinder) {
        return m_box_cylinder_cf;
    }

    return nullptr;
}