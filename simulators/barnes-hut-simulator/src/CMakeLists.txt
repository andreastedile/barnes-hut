add_library(barnes_hut_simulator_lib
        barnes_hut_simulator.cpp
        barnes_hut_simulator.h)

# If in the following lines I use PRIVATE (which I thought would be sufficient),
# barnes_hut_simulator.h does not find the #include inside.
# Using PUBLIC fixes the problem. Why?
target_link_libraries(barnes_hut_simulator_lib PUBLIC body_lib)
target_link_libraries(barnes_hut_simulator_lib PUBLIC quadtree_lib)
target_link_libraries(barnes_hut_simulator_lib PUBLIC physics_lib)
target_link_libraries(barnes_hut_simulator_lib PUBLIC simulation_step_lib)
if (WITH_TBB)
    target_link_libraries(barnes_hut_simulator_lib PRIVATE TBB::tbb)
else ()
    target_link_libraries(barnes_hut_simulator_lib PRIVATE OpenMP::OpenMP_CXX)
    if (DEBUG_OPENMP_BODY_UPDATE_FOR_LOOP)
        target_compile_definitions(barnes_hut_simulator_lib PUBLIC DEBUG_OPENMP_BODY_UPDATE_FOR_LOOP)
    endif ()
endif ()
target_link_libraries(barnes_hut_simulator_lib PRIVATE spdlog::spdlog)
