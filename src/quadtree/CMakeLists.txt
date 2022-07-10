add_library(quadtree_lib
        node.cpp
        node.h
        node_json.cpp
        quadtree.h
        quadtree.cpp)

target_link_libraries(quadtree_lib PUBLIC body_lib)
target_link_libraries(quadtree_lib PUBLIC Eigen3::Eigen)
target_link_libraries(quadtree_lib PUBLIC nlohmann_json::nlohmann_json)
target_link_libraries(quadtree_lib PRIVATE spdlog::spdlog)

target_include_directories(quadtree_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
