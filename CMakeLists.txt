cmake_minimum_required(VERSION 3.9)
project(barnes_hut CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# https://github.com/p-ranav/argparse
find_package(argparse 2.5.0 REQUIRED)

# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# https://json.nlohmann.me/integration/cmake/
find_package(nlohmann_json 3.10.5 REQUIRED)

find_package(Catch2 REQUIRED)

# https://github.com/oneapi-src/oneTBB/blob/master/cmake/README.md#tbbconfig---integration-of-binary-packages
find_package(TBB REQUIRED)

find_package(MPI REQUIRED)

# https://github.com/include-what-you-use/include-what-you-use
find_program(iwyu_path NAMES include-what-you-use iwyu)
if (iwyu_path)
    message("Found iwyu")
    set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${iwyu_path})
endif ()

add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(apps)
