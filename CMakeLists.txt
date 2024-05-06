cmake_minimum_required(VERSION 3.24)

project(box3d
        DESCRIPTION "3D Simulation developed by ECNU Robot lab")

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(box3d)

add_subdirectory(${CMAKE_SOURCE_DIR}/extern/glad)
add_subdirectory(${CMAKE_SOURCE_DIR}/extern/glfw)
add_subdirectory(${CMAKE_SOURCE_DIR}/extern/imgui)

add_subdirectory(test)
