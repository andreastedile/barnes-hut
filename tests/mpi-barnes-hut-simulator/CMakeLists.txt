add_executable(test-mpi-barnes-hut-simulator test_mpi_barnes_hut_simulator.cpp)

target_link_libraries(test-mpi-barnes-hut-simulator PRIVATE Catch2::Catch2WithMain mpi_barnes_hut_simulator_lib)
