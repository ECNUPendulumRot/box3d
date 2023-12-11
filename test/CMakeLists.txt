list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")

include(cli11)
include(libigl)
igl_include(glfw)

add_executable(test_mesh_read test_mesh_read.cpp)
add_executable(test_easy_world test_easy_world.cpp)
add_executable(test_bvh test_bvh.cpp)

add_executable(test_app test_app.cpp)

add_executable(test_gjk_distance test_gjk_distance.cpp)




set(B3D_SIMULATION_DIR ${CMAKE_SOURCE_DIR}/simulation)
set(B3D_MESH_DIR     "${B3D_SIMULATION_DIR}/mesh")
set(B3D_BODY_DEF_DIR "${B3D_SIMULATION_DIR}/body_definition")
set(B3D_SCENE_DIR    "${B3D_SIMULATION_DIR}/scene")

target_compile_definitions(test_mesh_read PRIVATE B3D_MESH_DIR="${B3D_MESH_DIR}")
target_compile_definitions(test_bvh       PRIVATE B3D_MESH_DIR="${B3D_MESH_DIR}" B3D_FIXTURE_DIR="${B3D_FIXTURE_DIR}")
target_compile_definitions(test_easy_world       PRIVATE B3D_MESH_DIR="${B3D_MESH_DIR}" B3D_FIXTURE_DIR="${B3D_FIXTURE_DIR}")
target_compile_definitions(test_gjk_distance PRIVATE B3D_MESH_DIR="${B3D_MESH_DIR}")


###################### b3simapp ######################
add_library(b3simapp
    src/b3_sim_app.cpp
    src/b3_test.cpp
    src/b3_gui_viewer.cpp
    include/b3_sim_app.hpp
    include/b3_test.hpp
    include/b3_gui_viewer.hpp

    include/test_sphere_dropping.hpp
)

target_include_directories(b3simapp
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(b3simapp
    PUBLIC box3d CLI11::CLI11 igl::glfw nlohmann_json::nlohmann_json
)

target_compile_definitions(b3simapp
    PRIVATE
    B3D_MESH_DIR="${B3D_MESH_DIR}"
    B3D_BODY_DEF_DIR="${B3D_BODY_DEF_DIR}"
    B3D_SCENE_DIR="${B3D_SCENE_DIR}"
)


###################### b3simapp ######################

target_include_directories(test_mesh_read PRIVATE box3d/include)
target_include_directories(test_bvh       PRIVATE box3d/include)
target_include_directories(test_easy_world       PRIVATE box3d/include)
target_include_directories(test_gjk_distance PRIVATE box3d/include)


#target_compile_definitions(test_mesh_read PRIVATE B3D_MESH_DIR="${B3D_MESH_DIR}")
#target_compile_definitions(test_bvh       PRIVATE B3D_MESH_DIR="${B3D_MESH_DIR}" B3D_FIXTURE_DIR="${B3D_FIXTURE_DIR}")
target_compile_definitions(test_easy_world       PRIVATE B3D_MESH_DIR="${B3D_MESH_DIR}" B3D_BODY_DEF_DIR="${B3D_BODY_DEF_DIR}")



#target_include_directories(test_mesh_read PRIVATE box3d/include include)
#target_include_directories(test_bvh       PRIVATE box3d/include include)
target_include_directories(test_easy_world  PRIVATE box3d/include include)
target_include_directories(test_app       PRIVATE box3d/include include)

#target_link_libraries(test_bvh       PUBLIC box3d CLI11::CLI11 igl::glfw)
#target_link_libraries(test_mesh_read PUBLIC box3d CLI11::CLI11 igl::glfw)
target_link_libraries(test_easy_world PUBLIC box3d CLI11::CLI11 igl::glfw nlohmann_json::nlohmann_json)


target_link_libraries(test_app PUBLIC box3d b3simapp )


target_link_libraries(test_gjk_distance PUBLIC box3d CLI11::CLI11 igl::glfw nlohmann_json::nlohmann_json)
