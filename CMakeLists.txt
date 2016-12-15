cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(feature_detectors)

find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(fpfh_test descriptor_sandbox/fpfh_test.cpp)
target_link_libraries(fpfh_test ${PCL_LIBRARIES})

add_executable(viewer visualization/viewer.cpp)
target_link_libraries(viewer ${PCL_LIBRARIES})
