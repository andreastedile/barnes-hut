add_executable(testlib
        testlib.cpp
        test_quadtree.cpp
        test_force.cpp
        test_body.cpp
        test_serialization.cpp
        test_deserialization.cpp
        test_eigen.cpp)

target_link_libraries(testlib PRIVATE barnes_hut_lib simulation_lib Catch2::Catch2)

# https://github.com/catchorg/Catch2/blob/devel/docs/cmake-integration.md
include(CTest)
include(Catch)
catch_discover_tests(testlib)
