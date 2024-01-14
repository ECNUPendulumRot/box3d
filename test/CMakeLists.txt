list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")

include(cli11)
include(libigl)
igl_include(glfw)




###################### b3simapp ######################

set(TEST_HEADER_FILES
    include/b3_sim_app.hpp
    include/b3_test.hpp
    include/b3_gui_viewer.hpp
        include/b3_category_menu.hpp

)

set(TEST_SOURCE_FILES
    test_app.cpp
    src/b3_sim_app.cpp
    src/b3_test.cpp
    src/b3_gui_viewer.cpp
    src/test_box_view.cpp
)

add_executable(
    test_app
    ${TEST_HEADER_FILES}
    ${TEST_SOURCE_FILES}
)

add_executable(
    test_sphere_view_data
    test_sphere_view_data.cpp
)

target_include_directories(test_app
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(test_app
    PUBLIC box3d CLI11::CLI11 igl::glfw nlohmann_json::nlohmann_json
)

target_link_libraries(test_sphere_view_data
    PUBLIC box3d igl::glfw
)