set(B3D_SOURCE_FILES
    src/dynamics/library.cpp)

set(B3D_HEADER_FILES
    include/box3d.h
    include/dynamics/library.h)

add_library(box3d ${B3D_SOURCE_FILES} ${B3D_HEADER_FILES})

target_include_directories(box3d
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
