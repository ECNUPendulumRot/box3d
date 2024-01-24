list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")

include(cli11)
include(libigl)
igl_include(glfw)




###################### b3simapp ######################

set(TEST_HEADER_FILES
        include/sim_app.hpp
        include/scene_test.hpp
        include/gui_viewer.hpp
        include/category_menu.hpp
        include/unit_test.hpp
        include/test.hpp

)

set(TEST_SOURCE_FILES
    test_app.cpp
        src/sim_app.cpp
        src/gui_viewer.cpp
    src/test_box_view.cpp
        src/unit_test_sphere_collide.cpp
        src/unit_test_box_collide.cpp
        src/test.cpp
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
    PUBLIC box3d CLI11::CLI11 igl::glfw nlohmann_json::nlohmann_json
)

target_link_libraries(
    test_all_class_size
    PUBLIC box3d
)

target_link_libraries(
    test_sphere_manifold
    PUBLIC box3d igl::glfw
)