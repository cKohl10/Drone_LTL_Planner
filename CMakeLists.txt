cmake_minimum_required(VERSION 3.5)
project(main)

# # Find the Triangle library
# find_library(TRIANGLE_LIBRARY triangle PATHS ~/triangle)

# # Check if the Triangle library was found
# if (NOT TRIANGLE_LIBRARY)
#     message(FATAL_ERROR "Triangle library not found")
# endif()


find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})

add_executable(main main.cpp)
target_link_libraries(main ${OMPL_LIBRARIES})
