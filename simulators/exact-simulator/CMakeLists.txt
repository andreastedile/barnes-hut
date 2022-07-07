add_subdirectory(src)

add_executable(exact-simulator exact_simulator_app.cpp)
target_link_libraries(exact-simulator PRIVATE utils_lib)
target_link_libraries(exact-simulator PRIVATE exact_simulator_lib)
