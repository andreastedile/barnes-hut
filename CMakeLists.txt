cmake_minimum_required(VERSION 3.9)
project(barnes_hut CXX)

set(CMAKE_CXX_STANDARD 17)

# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# https://json.nlohmann.me/integration/cmake/
find_package(nlohmann_json 3.10.5 REQUIRED)

add_subdirectory(src)
add_subdirectory(apps)
