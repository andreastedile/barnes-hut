add_library(data_transfer_lib
        body_deserialization.cpp
        body_deserialization.h
        body_serialization.cpp
        body_serialization.h
        mpi_datatypes.cpp
        mpi_datatypes.h
        quadtree_deserialization.cpp
        quadtree_deserialization.h
        quadtree_serialization.cpp
        quadtree_serialization.h)

target_link_libraries(data_transfer_lib PUBLIC body_lib)
target_link_libraries(data_transfer_lib PUBLIC quadtree_lib)
target_link_libraries(data_transfer_lib PRIVATE Eigen3::Eigen)
if (WITH_TBB)
    target_link_libraries(data_transfer_lib PRIVATE TBB::tbb)
else ()
    target_link_libraries(data_transfer_lib PRIVATE OpenMP::OpenMP_CXX)
endif ()

target_include_directories(data_transfer_lib INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
