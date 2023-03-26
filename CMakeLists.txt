cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(Pipeline)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_executable (Pipeline Pipeline.cpp)
target_link_libraries (Pipeline ${PCL_LIBRARIES})
