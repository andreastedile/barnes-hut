add_executable(test-quadtree test_quadtree.cpp)

target_link_libraries(test-quadtree PRIVATE Catch2::Catch2WithMain quadtree_lib)
