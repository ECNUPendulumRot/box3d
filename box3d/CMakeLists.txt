set(B3D_HEADER_FILES
        include/box3d.hpp

        include/common/b3_types.hpp
        include/common/b3_allocator.hpp
        include/common/b3_common.hpp

        include/collision/b3_bvh.hpp
        include/collision/b3_aabb.hpp
        include/collision/b3_fixture.hpp
        include/collision/b3_rigid_fixture.hpp
        include/collision/b3_broad_phase.hpp

        # include/collision/b3_gjk_distance.hpp
        # include/collision/b3_distance_proxy.hpp
        # include/collision/b3_collision.hpp
        include/collision/b3_contact_manager.hpp
        include/collision/b3_contact.hpp


        include/dynamics/b3_body_rigid.hpp
        include/dynamics/b3_pose.hpp
        include/dynamics/b3_body.hpp
        include/dynamics/b3_inertia.hpp
        include/dynamics/b3_world.hpp
        include/dynamics/b3_body_def.hpp
        include/dynamics/b3_body_affine.hpp

        include/geometry/b3_mesh.hpp
        include/geometry/b3_geometry.hpp

        include/utils/b3_io.hpp
        include/utils/b3_log.hpp
        include/utils/b3_json.hpp
        include/utils/b3_timer.hpp

        include/math/b3_matrix.hpp
        include/math/b3_vector.hpp
        include/math/b3_min_max.hpp
        include/math/b3_math.hpp


        include/solver/b3_solver.hpp
        include/geometry/b3_shape.hpp

)

set(B3D_SOURCE_FILES

        src/collision/b3_bvh.cpp
        src/collision/b3_aabb.cpp
        src/collision/b3_fixture.cpp
        src/collision/b3_broad_phase.cpp
        # src/collision/b3_gjk_distance.cpp
        # src/collision/b3_distance_proxy.cpp
        # src/collision/b3_collision.cpp
        src/collision/b3_contact.cpp
        src/collision/b3_contact_manager.cpp


        src/dynamics/b3_body_rigid.cpp
        src/dynamics/b3_pose.cpp
        src/dynamics/b3_inertia.cpp
        src/dynamics/b3_world.cpp
        src/dynamics/b3_body.cpp
        src/dynamics/b3_body_affine.cpp

        src/geometry/b3_mesh.cpp

        src/utils/b3_io.cpp
        src/utils/b3_log.cpp
        src/geometry/b3_shape.cpp

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