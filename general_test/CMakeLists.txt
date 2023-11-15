list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")

include(cli11)

add_executable(general_test general_test.cpp)

# Eigen
include(eigen)
target_link_libraries(general_test PUBLIC Eigen3::Eigen)