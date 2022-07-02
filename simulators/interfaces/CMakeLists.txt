add_library(interfaces_lib
        barnes_hut.cpp
        barnes_hut.h
        communication.cpp
        communication.h
        physics.cpp
        physics.h
        steppable.h)

target_link_libraries(interfaces_lib PRIVATE utils_lib)

target_include_directories(interfaces_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
