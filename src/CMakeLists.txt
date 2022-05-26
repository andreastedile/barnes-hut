add_library(barnes_hut_lib
        ${PROJECT_SOURCE_DIR}/include/barnes_hut/templates.h
        ${PROJECT_SOURCE_DIR}/include/barnes_hut/node.h
        node.cpp
        node_json.cpp
        ${PROJECT_SOURCE_DIR}/include/barnes_hut/body.h
        body.cpp
        ${PROJECT_SOURCE_DIR}/include/barnes_hut/force.h
        force.cpp
        ${PROJECT_SOURCE_DIR}/include/barnes_hut/simulation.h
        simulation.cpp
        simulation_json.cpp
        simple_simulator.cpp)

target_compile_features(barnes_hut_lib PRIVATE cxx_std_17)

target_link_libraries(barnes_hut_lib PRIVATE Eigen3::Eigen)
target_link_libraries(barnes_hut_lib PRIVATE nlohmann_json::nlohmann_json)

target_include_directories(barnes_hut_lib PUBLIC ${PROJECT_SOURCE_DIR}/include)
