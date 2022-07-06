add_executable(test-eigen test_eigen.cpp)

target_link_libraries(test-eigen PRIVATE Catch2::Catch2WithMain Eigen3::Eigen)
