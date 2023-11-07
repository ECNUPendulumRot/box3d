set(B3D_HEADER_FILES
        include/box3d.hpp

        include/dynamics/b3_rigid_body.hpp
        include/dynamics/b3_pose.hpp
        include/dynamics/b3_body.hpp
        include/dynamics/b3_inertia.hpp

        include/geometry/b3_mesh.hpp

        include/common/b3_types.hpp
        include/utils/b3_io.hpp
        include/utils/b3_log.hpp

        include/math/b3_matrix.hpp
        include/geometry/b3_geometry.hpp

        include/dynamics/b3_world.hpp
        include/collision/b3_bvh.hpp
        include/common/b3_allocator.hpp
        include/math/b3_vector.hpp
        include/common/b3_common.hpp
        include/math/b3_min_max.hpp
)

set(B3D_SOURCE_FILES
        src/dynamics/b3_rigid_body.cpp
        src/dynamics/b3_pose.cpp
        src/dynamics/b3_body.cpp
        src/dynamics/b3_inertia.cpp

        src/geometry/b3_mesh.cpp

        src/utils/b3_io.cpp
        src/utils/b3_log.cpp
        src/dynamics/b3_world.cpp
        src/collision/b3_bvh.cpp

)

# add third-party libraries
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/extern/cmake")

add_library(box3d ${B3D_SOURCE_FILES} ${B3D_HEADER_FILES})

target_include_directories(box3d
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Eigen
include(eigen)
target_link_libraries(box3d PUBLIC Eigen3::Eigen)

# spdlog
include(spdlog)
target_link_libraries(box3d PUBLIC spdlog::spdlog)
