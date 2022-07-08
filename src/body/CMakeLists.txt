add_library(body_lib
        body.cpp
        body.h
        body_json.cpp
        bounding_box.cpp
        bounding_box.h)

target_link_libraries(body_lib PUBLIC Eigen3::Eigen)
target_link_libraries(body_lib PUBLIC nlohmann_json::nlohmann_json)
if (WITH_TBB)
    target_link_libraries(body_lib PRIVATE TBB::tbb)
else ()
    target_link_libraries(body_lib PRIVATE OpenMP::OpenMP_CXX)
endif ()

if (DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS)
    target_compile_definitions(body_lib PUBLIC DEBUG_CONSTRUCTOR_AND_ASSIGNMENT_OPERATORS)
endif ()

target_include_directories(body_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
