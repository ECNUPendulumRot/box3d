###################### b3simapp ######################

set (TESTBED_SOURCE_FILES
        main.cpp
        imgui_impl_glfw.cpp
        imgui_impl_glfw.h
        imgui_impl_opengl3.cpp
        imgui_impl_opengl3.h
        draw.hpp
        draw.cpp
        settings.hpp
        include/test.hpp
        include/gl_render_triangles.hpp
        include/gl_common.hpp
        include/camera.hpp
        src/test.cpp
        src/test_box_rest.cpp
        src/gl_render_triangles.cpp
        src/camera.cpp
        src/test_sphere_drop.cpp
        src/test_sphere_friction.cpp
        src/test_sphere_drop_with_angular_velocity.cpp
        src/test_sphere_collide_plane.cpp
        src/test_sphere_slope.cpp
        src/test_sphere_rotating.cpp
        src/test_planet.cpp

        include/gl_render_lines.hpp
        src/gl_render_lines.cpp
        settings.cpp
)

add_executable(testbed ${TESTBED_SOURCE_FILES})

target_include_directories(testbed PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(testbed PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(testbed PUBLIC box3d glfw imgui glad sajson spdlog::spdlog)