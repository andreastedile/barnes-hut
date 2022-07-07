cmake_minimum_required(VERSION 3.9)
project(barnes_hut CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(argparse
        GIT_REPOSITORY https://github.com/p-ranav/argparse.git
        GIT_TAG v2.6)

FetchContent_Declare(Eigen3
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz)

# https://json.nlohmann.me/integration/cmake/#fetchcontent
FetchContent_Declare(nlohmann_json
        URL https://github.com/nlohmann/json/releases/download/v3.10.5/json.tar.xz)

# https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md#cmake-targets
FetchContent_Declare(Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG v3.0.1)

FetchContent_MakeAvailable(argparse)
FetchContent_MakeAvailable(Eigen3)
FetchContent_MakeAvailable(nlohmann_json)
FetchContent_MakeAvailable(Catch2)

find_package(TBB REQUIRED)
find_package(MPI REQUIRED)


add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(simulators)
