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
        include/camera.hpp
        include/gl_common.hpp
        include/test.hpp
        include/gl_render_lines.hpp
        include/gl_render_triangles.hpp

        settings.cpp
        src/camera.cpp
        src/gl_render_lines.cpp
        src/gl_render_points.cpp
        src/gl_render_triangles.cpp

        src/test.cpp
        src/test_box_rest.cpp

        src/test_sphere_drop.cpp
        src/test_box_stack.cpp
        src/test_sphere_billiards.cpp
        src/test_sphere_one_hit_tow.cpp
        src/test_two_hit_line.cpp
        src/test_sphere_extended_bernoulli.cpp
        src/test_sphere_cradle.cpp
        include/gl_render_object.hpp
        src/gl_render_object.cpp
        utils.hpp
        utils.cpp
        include/imgui_ext.hpp
        src/test_wall_break.cpp
        src/test_cube_propagation.cpp
        src/test_wall_rebound.cpp
        src/test_position_solver.cpp

        src/test_ball_geyser.cpp
)

add_executable(testbed ${TESTBED_SOURCE_FILES})

target_include_directories(testbed PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(testbed PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(testbed PUBLIC box3d glfw imgui glad sajson spdlog::spdlog nlohmann_json::nlohmann_json)