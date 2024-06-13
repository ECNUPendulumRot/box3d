list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")

set(B3D_HEADER_FILES

        include/box3d.hpp

        include/common/b3_draw.hpp
        include/common/b3_types.hpp
        include/common/b3_common.hpp
        include/common/b3_allocator.hpp
        include/common/b3_time_step.hpp
        include/common/b3_world_callback.hpp
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
        include/collision/b3_plane_cube_contact.hpp

        include/dynamics/b3_transform.hpp
        include/dynamics/b3_body.hpp
        include/dynamics/b3_world.hpp
        include/dynamics/b3_body_def.hpp
        include/dynamics/b3_island.hpp
        include/dynamics/b3_mass_property.hpp

        include/geometry/b3_shape.hpp
        include/geometry/b3_cube_shape.hpp
        include/geometry/b3_sphere_shape.hpp
        include/geometry/b3_plane_shape.hpp

        include/utils/b3_timer.hpp

        include/math/b3_vec3.hpp
        include/math/b3_vec2.hpp
        include/math/b3_min_max.hpp
        include/math/b3_math_op.hpp
        include/math/b3_quat.hpp
        include/math/b3_vec12.hpp
        include/math/b3_mat1212.hpp

        include/solver/b3_lemke.hpp
        include/solver/b3_solver.hpp
        include/solver/b3_solver_zhb.hpp
        include/solver/b3_contact_constraint.hpp
        include/solver/b3_solver_gr.hpp
        include/solver/b3_contact_solver.hpp
        include/solver/b3_contact_solver_zhb.hpp
        # include/collision/b3_distance.hpp
        # include/collision/b3_time_of_impact.hpp
)


set(B3D_SOURCE_FILES

        src/common/b3_block_allocator.cpp
        src/common/b3_draw.cpp

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
        src/collision/b3_plane_cube_contact.cpp

        src/dynamics/b3_world.cpp
        src/dynamics/b3_body.cpp
        src/dynamics/b3_island.cpp

        src/geometry/b3_shape.cpp
        src/geometry/b3_cube_shape.cpp
        src/geometry/b3_plane_shape.cpp
        src/geometry/b3_sphere_shape.cpp

        src/solver/b3_lemke.cpp
        src/solver/b3_solver.cpp
        src/solver/b3_solver_zhb.cpp
        src/solver/b3_solver_gr.cpp
        src/solver/b3_contact_solver.cpp
        src/collision/b3_collide.cpp
        src/solver/b3_contact_solver_zhb.cpp
        # src/collision/b3_distance.cpp
        # src/collision/b3_time_of_impact.cpp
)

# add third-party libraries
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")

add_library(box3d ${B3D_SOURCE_FILES} ${B3D_HEADER_FILES})

target_link_libraries(box3d PUBLIC spdlog::spdlog)

target_include_directories(box3d
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)