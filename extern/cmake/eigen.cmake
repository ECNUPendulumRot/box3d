
if(TARGET Eigen3::Eigen)
    return()
endif()

message(STATUS "Third-party: creating target 'Eigen3::Eigen'")

include(FetchContent)
FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG tags/3.4.0
)
FetchContent_MakeAvailable(eigen)
