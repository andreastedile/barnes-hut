add_library(simulation_lib
        simulated_body.cpp
        simulated_body.h
        simulated_body.cpp
        simulated_body_json.cpp
        simulation_step.cpp
        simulation_step.h
        exact_simulation_step.h
        exact_simulation_step_json.cpp
        barnes_hut_simulation_step.cpp
        barnes_hut_simulation_step.h
        barnes_hut_simulation_step_json.cpp
        simulation.cpp
        simulation.h
        local_exact_simulator.cpp
        local_exact_simulator_json.cpp
        local_exact_simulator.h
        local_barnes_hut_simulator.cpp
        local_barnes_hut_simulator_json.cpp
        local_barnes_hut_simulator.h)

target_include_directories(simulation_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

if (DEBUG_COPY_CONSTRUCTOR)
    target_compile_definitions(simulation_lib PRIVATE DEBUG_COPY_CONSTRUCTOR)
endif ()
if (DEBUG_MOVE_CONSTRUCTOR)
    target_compile_definitions(simulation_lib PRIVATE DEBUG_MOVE_CONSTRUCTOR)
endif ()
if (DEBUG_COPY_ASSIGNMENT_OPERATOR)
    target_compile_definitions(simulation_lib PRIVATE DEBUG_COPY_ASSIGNMENT_OPERATOR)
endif ()
if (DEBUG_MOVE_ASSIGNMENT_OPERATOR)
    target_compile_definitions(simulation_lib PRIVATE DEBUG_MOVE_ASSIGNMENT_OPERATOR)
endif ()

target_link_libraries(simulation_lib PRIVATE barnes_hut_lib)
target_link_libraries(simulation_lib PRIVATE Eigen3::Eigen)
target_link_libraries(simulation_lib PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(simulation_lib PRIVATE TBB::tbb)
