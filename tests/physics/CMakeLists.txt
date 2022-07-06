add_executable(test-force test_force.cpp)

target_link_libraries(test-force PRIVATE Catch2::Catch2WithMain physics_lib)
