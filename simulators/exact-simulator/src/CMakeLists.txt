add_library(exact_simulator_lib
        exact_simulator.cpp
        exact_simulator.h)

# If in the following lines I use PRIVATE (which I thought would be sufficient),
# exact_simulator.h does not find the #include inside.
# Using PUBLIC fixes the problem. Why?
target_link_libraries(exact_simulator_lib PUBLIC body_lib)
target_link_libraries(exact_simulator_lib PUBLIC physics_lib)
target_link_libraries(exact_simulator_lib PUBLIC simulation_step_lib)
if (WITH_TBB)
    target_link_libraries(exact_simulator_lib PRIVATE TBB::tbb)
else ()
    target_link_libraries(exact_simulator_lib PRIVATE OpenMP::OpenMP_CXX)
endif ()
