add_executable(testlib
        testlib.cpp
        test_quadtree.cpp
        test_force.cpp)

target_link_libraries(testlib PRIVATE barnes_hut_lib Catch2::Catch2)

# https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md
include(CTest)
include(Catch)
catch_discover_tests(testlib)
