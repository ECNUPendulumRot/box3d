list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")



include(libigl)
igl_include(glfw)
igl_include(imgui)

include(spdlog)
###################### b3simapp ######################

set(TEST_HEADER_FILES
        include/sim_app.hpp
        include/scene_test.hpp
        include/gui/gui_viewer.hpp
        include/gui/gui_test_menu.hpp
        include/unit_test.hpp
        include/test.hpp
        include/auxiliary_shape.hpp
        include/gui/gui_mesh_menu.hpp
        include/gui/imgui_ext.hpp
        include/gui/gui_title_bar.hpp

)

set(TEST_SOURCE_FILES
        test_app.cpp
        src/sim_app.cpp
        src/gui_viewer.cpp
        src/scene_test_sphere_stack_dropping.cpp
        # src/unit_test_box_collide.cpp
        # src/unit_test_dynamic_tree.cpp
        src/test.cpp
        # src/unit_test_sphere_view.cpp
        src/scene_test_sphere_cube_collide.cpp
        src/imgui_ext.cpp
        src/scene_test_plane_sphere_collide.cpp
        src/scene_test_cube_rest.cpp
        src/scene_test_cube_dropping.cpp
        src/scene_test_sphere_friction.cpp
        src/scene_test_cube_friction.cpp
        src/scene_test_cube_collision.cpp
        src/scene_test_sphere_stack.cpp
        src/scene_test_cube_stack.cpp
        src/scene_test_newtons_cradle.cpp
        src/scene_test_symmetry_collision.cpp
        src/scene_test_sphere_dropping_no_gravity.cpp
        src/scene_test_vs_sphere_collision.cpp
        src/scene_test_vs_box_rotation.cpp
        src/scene_test_vs_box_rotation_no_gravity.cpp
        src/scene_test_affine_cube_dropping.cpp
        src/scene_test_affine_ground.cpp
)

add_executable(
    test_app
    ${TEST_HEADER_FILES}
    ${TEST_SOURCE_FILES}
)

add_executable(
    test_all_class_size
    test_all_class_size.cpp
)

add_executable(
    test_block_allocator
    src/test_block_allocator.cpp
)

add_executable(
    test_sphere_manifold
    unit_test/test_sphere_manifold.cpp
)

target_include_directories(test_app
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(test_app
    PUBLIC box3d igl::glfw spdlog::spdlog igl::imgui
)

target_link_libraries(
    test_all_class_size
    PUBLIC box3d
)

target_link_libraries(
    test_block_allocator
    PUBLIC box3d
)

target_link_libraries(
    test_sphere_manifold
    PUBLIC box3d igl::glfw
)