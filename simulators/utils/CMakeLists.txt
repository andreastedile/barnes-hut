add_library(utils_lib
        loader.cpp
        loader.h
        power_of_four.h)

target_link_libraries(utils_lib PUBLIC body_lib)

target_include_directories(utils_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
