add_library(barnes_hut_lib
        templates.h
        body.h
        body.cpp
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

if (DEBUG_COPY_CONSTRUCTOR)
    target_compile_definitions(barnes_hut_lib PRIVATE DEBUG_COPY_CONSTRUCTOR)
endif()
if (DEBUG_MOVE_CONSTRUCTOR)
    target_compile_definitions(barnes_hut_lib PRIVATE DEBUG_MOVE_CONSTRUCTOR)
endif()
if (DEBUG_COPY_ASSIGNMENT_OPERATOR)
    target_compile_definitions(barnes_hut_lib PRIVATE DEBUG_COPY_ASSIGNMENT_OPERATOR)
endif()
if (DEBUG_MOVE_ASSIGNMENT_OPERATOR)
    target_compile_definitions(barnes_hut_lib PRIVATE DEBUG_MOVE_ASSIGNMENT_OPERATOR)
endif()

target_link_libraries(barnes_hut_lib PRIVATE Eigen3::Eigen)
target_link_libraries(barnes_hut_lib PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(barnes_hut_lib PRIVATE TBB::tbb)