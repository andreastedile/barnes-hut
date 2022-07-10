add_library(utils_lib
        loader.cpp
        loader.h
        persistence.h
        power_of_four.h
        step_format.h)

target_link_libraries(utils_lib PUBLIC body_lib)
target_link_libraries(utils_lib PRIVATE nlohmann_json::nlohmann_json)

target_include_directories(utils_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
