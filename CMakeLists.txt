cmake_minimum_required(VERSION 3.0)

set(CXX_BUILD_TYPE RELEASE)
find_package(PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

set(SRC bin.cc 
        ground_segmentation.cc
        segment.cc
        viewer.cc
        main.cpp
        )
add_executable(a ${SRC})

target_link_libraries(a ${PCL_LIBRARIES} pthread)