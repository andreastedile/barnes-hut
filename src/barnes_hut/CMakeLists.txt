add_library(barnes_hut_lib
        templates.h
        body.h
        body.cpp
        node.h
        node.cpp
        node_json.cpp
        force.h
        force.cpp)

target_include_directories(barnes_hut_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(barnes_hut_lib PRIVATE Eigen3::Eigen)
target_link_libraries(barnes_hut_lib PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(barnes_hut_lib PRIVATE TBB::tbb)