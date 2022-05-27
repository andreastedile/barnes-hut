add_library(simulation_lib
        simulation.cpp
        simulation.h
        simulation_json.cpp
        simple_simulator.cpp)

target_include_directories(simulation_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(simulation_lib PRIVATE barnes_hut_lib)
target_link_libraries(simulation_lib PRIVATE Eigen3::Eigen)
target_link_libraries(simulation_lib PRIVATE nlohmann_json::nlohmann_json)
