cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(TryTrajectory)
include(FetchContent)

FetchContent_Declare(
    matplotplusplus
    GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus.git
    GIT_TAG master
)

FetchContent_MakeAvailable(matplotplusplus)

add_executable(main main.cpp trajectory_RK4.cpp)
target_link_libraries(main PRIVATE matplot)
