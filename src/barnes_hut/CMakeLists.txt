add_library(barnes_hut_lib
        templates.h
        body.h
        body.cpp
        bounding_box.cpp
        bounding_box.h
        quadtree.cpp
        quadtree.h
        body_json.cpp
        node.h
        node.cpp
        node_json.cpp
        force.h
        force.cpp
        body_update.cpp
        body_update.h
        ../eigen_json.h)

target_include_directories(barnes_hut_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if (DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS)
    target_compile_definitions(barnes_hut_lib PRIVATE DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS)
endif()

target_link_libraries(barnes_hut_lib PRIVATE Eigen3::Eigen)
target_link_libraries(barnes_hut_lib PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(barnes_hut_lib PRIVATE TBB::tbb)