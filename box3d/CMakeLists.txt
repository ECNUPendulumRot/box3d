set(B3D_HEADER_FILES
        include/box3d.hpp

        include/common/b3_types.hpp
        include/common/b3_allocator.hpp
        include/common/b3_common.hpp
        include/common/b3_time_step.hpp

        include/collision/b3_bvh.hpp
        include/collision/b3_aabb.hpp
        include/collision/b3_fixture.hpp
        include/collision/b3_broad_phase.hpp

        include/collision/b3_contact_manager.hpp
        include/collision/b3_contact.hpp
        include/collision/b3_collision.hpp
        include/collision/b3_cube_contact.hpp
        include/collision/b3_sphere_contact.hpp


        include/dynamics/b3_pose.hpp
        include/dynamics/b3_body.hpp
        include/dynamics/b3_inertia.hpp
        include/dynamics/b3_world.hpp
        include/dynamics/b3_body_def.hpp
        include/dynamics/b3_island.hpp

        include/geometry/b3_mesh.hpp
        include/geometry/b3_geometry.hpp
        include/geometry/b3_shape.hpp
        include/geometry/b3_cube_shape.hpp
        include/geometry/b3_sphere_shape.hpp

        include/utils/b3_io.hpp
        include/utils/b3_log.hpp
        include/utils/b3_json.hpp
        include/utils/b3_timer.hpp

        include/math/b3_matrix_operation.hpp
        include/math/b3_vector.hpp
        include/math/b3_min_max.hpp
        include/math/b3_math.hpp


        include/solver/b3_solver.hpp
        include/solver/b3_si_solver.hpp
        include/solver/b3_contact_constraint.hpp

)

set(B3D_SOURCE_FILES

        src/collision/b3_bvh.cpp
        src/collision/b3_aabb.cpp
        src/collision/b3_fixture.cpp
        src/collision/b3_broad_phase.cpp
        src/collision/b3_contact.cpp
        src/collision/b3_contact_manager.cpp

        src/collision/b3_sphere_contact.cpp
        src/collision/b3_collide_sphere.cpp
        src/collision/b3_collide_cube.cpp

        src/dynamics/b3_pose.cpp
        src/dynamics/b3_inertia.cpp
        src/dynamics/b3_world.cpp
        src/dynamics/b3_body.cpp
        src/dynamics/b3_island.cpp

        src/geometry/b3_mesh.cpp
        src/geometry/b3_cube_shape.cpp
        src/geometry/b3_sphere_shape.cpp

        src/utils/b3_io.cpp
        src/utils/b3_log.cpp

        src/solver/b3_si_solver.cpp

)

# add third-party libraries
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")

add_library(box3d ${B3D_SOURCE_FILES} ${B3D_HEADER_FILES})

target_include_directories(box3d
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Eigen
include(eigen)
target_link_libraries(box3d PUBLIC Eigen3::Eigen)

# spdlog
include(spdlog)
target_link_libraries(box3d PUBLIC spdlog::spdlog)

# nlohmann_json
include(json)
target_link_libraries(box3d PUBLIC nlohmann_json::nlohmann_json)

# igl
include(libigl)

igl_include(glfw)
igl_include(imgui)
target_link_libraries(box3d PUBLIC igl::glfw igl::imgui)