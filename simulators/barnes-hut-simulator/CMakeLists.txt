add_subdirectory(src)

add_executable(barnes-hut-simulator barnes_hut_simulator_app.cpp)

target_link_libraries(barnes-hut-simulator PRIVATE nlohmann_json::nlohmann_json)
target_link_libraries(barnes-hut-simulator PRIVATE spdlog::spdlog)
target_link_libraries(barnes-hut-simulator PRIVATE utils_lib)
target_link_libraries(barnes-hut-simulator PRIVATE barnes_hut_simulator_lib)
target_link_libraries(barnes-hut-simulator PRIVATE body_lib)
