add_library(mpi_barnes_hut_simulator_lib
        mpi_barnes_hut_simulator.cpp
        mpi_barnes_hut_simulator.h)

# If in the following lines I use PRIVATE (which I thought would be sufficient),
# mpi_barnes_hut_simulator.h does not find the #include inside.
# Using PUBLIC fixes the problem. Why?
target_link_libraries(mpi_barnes_hut_simulator_lib PUBLIC body_lib)
target_link_libraries(mpi_barnes_hut_simulator_lib PUBLIC quadtree_lib)
target_link_libraries(mpi_barnes_hut_simulator_lib PUBLIC physics_lib)
target_link_libraries(mpi_barnes_hut_simulator_lib PUBLIC simulation_step_lib)
target_link_libraries(mpi_barnes_hut_simulator_lib PUBLIC data_transfer_lib)
target_link_libraries(mpi_barnes_hut_simulator_lib PUBLIC gather_lib)
if (WITH_TBB)
    target_link_libraries(mpi_barnes_hut_simulator_lib PRIVATE TBB::tbb)
else ()
    target_link_libraries(mpi_barnes_hut_simulator_lib PRIVATE OpenMP::OpenMP_CXX)
endif ()

target_include_directories(mpi_barnes_hut_simulator_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
