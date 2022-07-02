add_library(simulation_lib
        simulation_step.cpp
        simulation_step.h
        exact_simulation_step.cpp
        exact_simulation_step.h
        exact_simulation_step_json.cpp
        barnes_hut_simulation_step.cpp
        barnes_hut_simulation_step.h
        barnes_hut_simulation_step_json.cpp
        mpi_barnes_hut_simulator.cpp
        mpi_barnes_hut_simulator_json.cpp
        mpi_barnes_hut_simulator.h
        simulation.cpp
        simulation.h
        local_exact_simulator.cpp
        local_exact_simulator_json.cpp
        local_exact_simulator.h
        local_barnes_hut_simulator.cpp
        local_barnes_hut_simulator_json.cpp
        local_barnes_hut_simulator.h
        mpi_datatypes.cpp
        mpi_datatypes.h
        serialization.cpp
        serialization.h
        deserialization.cpp
        deserialization.h
        templates.h
        ../eigen_json.h)

target_include_directories(simulation_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if (DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS)
    target_compile_definitions(simulation_lib PRIVATE DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS)
endif ()

target_link_libraries(simulation_lib PRIVATE barnes_hut_lib)
target_link_libraries(simulation_lib PRIVATE Eigen3::Eigen)
target_link_libraries(simulation_lib PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(simulation_lib PRIVATE TBB::tbb)
target_link_libraries(simulation_lib PUBLIC MPI::MPI_CXX)
