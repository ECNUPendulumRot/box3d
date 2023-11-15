cmake_minimum_required(VERSION 3.26)

project(box3d
        DESCRIPTION "3D Simulation developed by ECNU Robot lab")

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(box3d)

add_subdirectory(test)

add_subdirectory(general_test)