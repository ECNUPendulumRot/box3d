set(B3D_HEADER_FILES
        include/box3d.hpp

        include/dynamics/b3_rigid_body.hpp
        include/dynamics/b3_pose.hpp
        include/dynamics/b3_body.hpp
        include/dynamics/b3_inertia.hpp

        include/geometry/b3_mesh.hpp

        include/common/b3_types.hpp
        include/utils/b3_io.hpp
        include/utils/b3_log.hpp

        include/math/b3_matrix.hpp
        include/geometry/b3_geometry.hpp

        include/dynamics/b3_world.hpp
        include/collision/b3_bvh.hpp
        include/common/b3_allocator.hpp
        include/math/b3_vector.hpp
        include/common/b3_common.hpp
        include/math/b3_min_max.hpp
        include/math/b3_math.hpp
        include/collision/b3_fixture.hpp
        include/collision/b3_rigid_fixture.hpp
        include/collision/b3_aabb.hpp
        include/dynamics/b3_body_def.hpp
        include/utils/b3_json.hpp
        include/utils/b3_timer.h
        include/utils/b3_gui_viewer.hpp
        include/collision/b3_broad_phase.hpp
        #include/utils/b3_igl_viewer_ext.hpp
        #include/utils/b3_imgui_ext.hpp


        include/collision/b3_distance.hpp
        include/collision/b3_distance_proxy.hpp
        include/dynamics/b3_contact_manager.hpp
        include/dynamics/b3_contact.hpp
        include/collision/b3_collision.hpp
)

set(B3D_SOURCE_FILES
        src/dynamics/b3_rigid_body.cpp
        src/dynamics/b3_pose.cpp
        src/dynamics/b3_inertia.cpp

        src/geometry/b3_mesh.cpp

        src/utils/b3_io.cpp
        src/utils/b3_log.cpp
        src/dynamics/b3_world.cpp
        src/collision/b3_bvh.cpp
        src/collision/b3_aabb.cpp
        src/dynamics/b3_body_def.cpp
        src/utils/b3_gui_viewer.cpp
        src/collision/b3_fixture.cpp
        src/collision/b3_broad_phase.cpp
        src/dynamics/b3_body.cpp
        #src/utils/b3_igl_viewer_ext.cpp
        #src/utils/b3_ui_menu.cpp
        #src/utils/b3_imgui_ext.cpp

        src/collision/b3_distance.cpp
        src/collision/b3_distance_proxy.cpp
        src/dynamics/b3_contact.cpp
        src/dynamics/b3_contact_manager.cpp
        src/collision/b3_collision.cpp

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