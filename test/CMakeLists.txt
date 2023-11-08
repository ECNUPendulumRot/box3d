list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")

include(cli11)


add_executable(test_mesh_read test_mesh_read.cpp)
add_executable(test_bvh test_bvh.cpp)

set(B3D_MESH_DIR "${CMAKE_SOURCE_DIR}/mesh")

target_compile_definitions(test_mesh_read PRIVATE B3D_MESH_DIR="${B3D_MESH_DIR}")
target_compile_definitions(test_bvh       PRIVATE B3D_MESH_DIR="${B3D_MESH_DIR}")

include(libigl)

igl_include(glfw)

target_include_directories(test_mesh_read PRIVATE box3d/include)
target_include_directories(test_bvh       PRIVATE box3d/include)

target_link_libraries(test_bvh       PUBLIC box3d CLI11::CLI11 igl::glfw)
target_link_libraries(test_mesh_read PUBLIC box3d CLI11::CLI11 igl::glfw)