add_library(physics_lib
        body_update.cpp
        body_update.h
        force.cpp
        force.h)

target_link_libraries(physics_lib PUBLIC body_lib)
target_link_libraries(physics_lib PUBLIC quadtree_lib)
if (WITH_TBB)
    target_link_libraries(physics_lib PRIVATE TBB::tbb)
else ()
    target_link_libraries(physics_lib PRIVATE OpenMP::OpenMP_CXX)
endif ()
target_link_libraries(physics_lib PRIVATE spdlog::spdlog)

if (DEBUG_COMPUTE_GRAVITATIONAL_FORCE)
    target_compile_definitions(physics_lib PUBLIC DEBUG_COMPUTE_GRAVITATIONAL_FORCE)
endif ()

target_include_directories(physics_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)