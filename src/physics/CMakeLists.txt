add_library(physics_lib
        body_update.cpp
        body_update.h
        force.cpp
        force.h)

target_link_libraries(physics_lib PUBLIC body_lib)
target_link_libraries(physics_lib PUBLIC quadtree_lib)

target_include_directories(physics_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)