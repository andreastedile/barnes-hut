add_library(gather_lib
        bodies_gathering.cpp
        bodies_gathering.h
        quadtree_gathering.cpp
        bodies_gathering.h)

target_link_libraries(gather_lib PUBLIC body_lib)
target_link_libraries(gather_lib PUBLIC quadtree_lib)

target_link_libraries(gather_lib PRIVATE data_transfer_lib)

target_link_libraries(gather_lib PRIVATE MPI::MPI_CXX)

target_include_directories(gather_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
