add_library(simulation_step_lib
        barnes_hut_simulation_step.cpp
        barnes_hut_simulation_step.h
        barnes_hut_simulation_step_json.cpp
        simulation_step.cpp
        simulation_step.h
        simulation_step_json.cpp)

target_link_libraries(simulation_step_lib PUBLIC body_lib)
target_link_libraries(simulation_step_lib PUBLIC quadtree_lib)
target_link_libraries(simulation_step_lib PUBLIC Eigen3::Eigen)
target_link_libraries(simulation_step_lib PUBLIC nlohmann_json::nlohmann_json)
target_link_libraries(simulation_step_lib PRIVATE spdlog::spdlog)

if (DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS)
    target_compile_definitions(simulation_step_lib PUBLIC DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS)
endif ()

target_include_directories(simulation_step_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
