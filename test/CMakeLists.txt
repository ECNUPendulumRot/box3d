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
        include/gui/test_menu.hpp
        include/unit_test.hpp
        include/test.hpp
        include/auxiliary_shape.hpp
        include/gui/mesh_menu.hpp
        include/gui/imgui_ext.hpp
        include/gui/title_bar.hpp

)

set(TEST_SOURCE_FILES
        test_app.cpp
        src/sim_app.cpp
        src/gui_viewer.cpp
        src/test_box_view.cpp
        src/test_sphere_stack.cpp
        src/unit_test_sphere_collide.cpp
        src/unit_test_box_collide.cpp
        src/unit_test_dynamic_tree.cpp
        src/test.cpp
        src/unit_test_sphere_view.cpp
        src/test_sphere_cube_collide.cpp
        src/imgui_ext.cpp
        src/unit_test_ground_view.cpp
        src/unit_test_plane_sphere_collide.cpp
        src/unit_test_plane_box_collide.cpp
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
    test_sphere_manifold
    PUBLIC box3d igl::glfw
)