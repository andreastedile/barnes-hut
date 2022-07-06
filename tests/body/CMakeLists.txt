add_executable(test-bounding-box test_bounding_box.cpp)

target_link_libraries(test-bounding-box PRIVATE Catch2::Catch2WithMain body_lib)
