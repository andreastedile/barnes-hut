add_executable(test-loader test_loader.cpp)

target_link_libraries(test-loader PRIVATE Catch2::Catch2WithMain utils_lib)

target_compile_definitions(test-loader PUBLIC BODY_TWO="${CMAKE_CURRENT_SOURCE_DIR}/body-2.txt")
target_compile_definitions(test-loader PUBLIC BODY_BAD="${CMAKE_CURRENT_SOURCE_DIR}/body-bad.txt")
