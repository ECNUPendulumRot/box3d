list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")

set(B3D_HEADER_FILES
        include/box3d.hpp

        include/common/b3_types.hpp
        include/common/b3_allocator.hpp
        include/common/b3_common.hpp
        include/common/b3_time_step.hpp
        include/common/b3_block_allocator.hpp

        include/collision/b3_bvh.hpp
        include/collision/b3_aabb.hpp
        include/collision/b3_fixture.hpp
        include/collision/b3_broad_phase.hpp

        include/collision/b3_contact_manager.hpp
        include/collision/b3_contact.hpp
        include/collision/b3_collision.hpp
        include/collision/b3_cube_contact.hpp
        include/collision/b3_sphere_contact.hpp
        include/collision/b3_cube_sphere_contact.hpp
        include/collision/b3_plane_sphere_contact.hpp
        include/collision/b3_plane_cone_contact.hpp


        include/dynamics/b3_transform.hpp
        include/dynamics/b3_body.hpp
        include/dynamics/b3_world.hpp
        include/dynamics/b3_body_def.hpp
        include/dynamics/b3_island.hpp
        include/dynamics/b3_mass_property.hpp
        include/dynamics/b3_transform_util.hpp

        include/geometry/b3_shape.hpp
        include/geometry/b3_cube_shape.hpp
        include/geometry/b3_sphere_shape.hpp
        include/geometry/b3_plane_shape.hpp
        include/geometry/b3_cone_shape.hpp
        include/geometry/b3_cylinder_shape.hpp

        include/utils/b3_timer.hpp

        include/math/b3_vec3.hpp
        include/math/b3_vec2.hpp
        include/math/b3_min_max.hpp
        include/math/b3_math_op.hpp


        include/solver/b3_solver.hpp
        include/solver/b3_contact_constraint.hpp
        include/collision/b3_plane_cube_contact.hpp

        include/math/b3_quat.hpp
        include/common/b3_draw.hpp


        include/collision/b3_dispatcher.hpp
        include/collision/b3_persistent_manifold.hpp
        include/collision/b3_manifold_result.hpp
        include/collision/b3_persistent_manifold_point.hpp
        include/collision/b3_collision_config.hpp
        include/collision/b3_collision_algorithm_create_func.hpp
        include/collision/algorithm/b3_collision_algorithm.hpp
        include/collision/algorithm/b3_sphere_sphere_collision_algorithm.hpp
        include/collision/algorithm/b3_box_box_detector.hpp
        include/collision/algorithm/b3_box_box_collision_algorithm.hpp
        include/collision/algorithm/b3_sphere_box_collision_algorithm.hpp
        include/collision/algorithm/b3_plane_sphere_collision_algorithm.hpp
        include/collision/algorithm/b3_plane_box_collision_algorithm.hpp
        include/collision/algorithm/b3_plane_cone_collision_algorithm.hpp
        include/collision/algorithm/b3_box_cone_collision_algorithm.hpp
        include/collision/algorithm/b3_box_cylinder_collision_algorithm.hpp

        include/collision/b3_cube_cone_contact.hpp

        include/collision/gjk/b3_gjk_pair_detector.hpp
        include/collision/gjk/b3_voronoi_simplex_solver.hpp
        include/collision/gjk/b3_penetration_depth_solver.hpp
        include/collision/gjk/b3_gjk_epa_solver2.hpp

        include/solver/b3_solver_body.hpp
        include/solver/b3_solver_lxj.hpp
        include/solver/b3_solver_constraint.hpp

        include/collision/b3_octree.hpp
        include/dynamics/constraint/b3_constraint_base.hpp
        include/dynamics/constraint/b3_point_point_constraint.hpp
        include/dynamics/constraint/b3_jacobian_entry.hpp

        include/collision/b3_cube_cylinder_contact.hpp
)

set(B3D_SOURCE_FILES

        src/collision/b3_bvh.cpp
        src/collision/b3_aabb.cpp
        src/collision/b3_fixture.cpp
        src/collision/b3_broad_phase.cpp
        src/collision/b3_contact.cpp
        src/collision/b3_contact_manager.cpp

        src/collision/b3_sphere_contact.cpp
        src/collision/b3_cube_contact.cpp
        src/collision/b3_collide_sphere.cpp
        src/collision/b3_collide_cube.cpp
        src/collision/b3_collide_plane.cpp
        src/collision/b3_cube_sphere_contact.cpp
        src/collision/b3_plane_sphere_contact.cpp
        src/collision/b3_collide_cone.cpp
        src/collision/b3_plane_cone_contact.cpp

        src/common/b3_block_allocator.cpp

        src/dynamics/b3_world.cpp
        src/dynamics/b3_body.cpp
        src/dynamics/b3_island.cpp

        src/geometry/b3_shape.cpp
        src/geometry/b3_cube_shape.cpp
        src/geometry/b3_sphere_shape.cpp
        src/geometry/b3_plane_shape.cpp
        src/geometry/b3_cone_shape.cpp
        src/geometry/b3_cylinder_shape.cpp

        src/solver/b3_solver.cpp
        src/geometry/b3_shape.cpp
        src/collision/b3_plane_cube_contact.cpp
        src/common/b3_draw.cpp

        src/collision/b3_collision_config.cpp
        src/collision/b3_dispatcher.cpp
        src/collision/b3_manifold_result.cpp
        src/collision/b3_persistent_manifold.cpp
        src/collision/algorithm/b3_sphere_sphere_collision_algorithm.cpp
        src/collision/algorithm/b3_box_box_detector.cpp
        src/collision/algorithm/b3_box_box_collision_algorithm.cpp
        src/collision/algorithm/b3_sphere_box_collision_algorithm.cpp
        src/collision/algorithm/b3_plane_sphere_collision_algorithm.cpp
        src/collision/algorithm/b3_plane_box_collision_algorithm.cpp
        src/collision/algorithm/b3_plane_cone_collision_algorithm.cpp
        src/collision/algorithm/b3_box_cone_collision_algorithm.cpp
        src/collision/algorithm/b3_box_cylinder_cllision_algorithm.cpp

        src/collision/b3_cube_cone_contact.cpp

        src/collision/gjk/b3_gjk_pair_detector.cpp
        src/collision/gjk/b3_voronoi_simplex_solver.cpp
        src/collision/gjk/b3_penetration_depth_solver.cpp
        src/collision/gjk/b3_gjk_epa_solver2.cpp

        src/solver/b3_solver_lxj.cpp

        src/collision/b3_octree.cpp

        src/dynamics/constraints/b3_constraint_base.cpp
        src/dynamics/constraints/b3_point_point_constraint.cpp

        src/collision/b3_cube_cylinder_contact.cpp
)

# add third-party libraries
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")

add_library(box3d ${B3D_SOURCE_FILES} ${B3D_HEADER_FILES})

target_link_libraries(box3d PUBLIC spdlog::spdlog)

target_include_directories(box3d
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)