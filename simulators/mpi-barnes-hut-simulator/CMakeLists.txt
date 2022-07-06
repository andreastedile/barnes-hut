add_subdirectory(src)

add_executable(mpi-barnes-hut-simulator mpi_barnes_hut_simulator_app.cpp)
target_link_libraries(mpi-barnes-hut-simulator PRIVATE TBB::tbb)
target_link_libraries(mpi-barnes-hut-simulator PRIVATE utils_lib)
target_link_libraries(mpi-barnes-hut-simulator PRIVATE MPI::MPI_CXX)
target_link_libraries(mpi-barnes-hut-simulator PRIVATE mpi_barnes_hut_simulator_lib)
