cmake_minimum_required(VERSION 3.12)
project(main)

# Include OMPL
find_package(ompl REQUIRED)

# Check if the OMPL library was found
if (NOT OMPL_FOUND)
    message(FATAL_ERROR "OMPL not found!")
endif()

# Find OpenCV
find_package(OpenCV REQUIRED)

# Check if the OpenCV library was found
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found!")
endif()

# Include the OMPL directories and the src directory
include_directories(${OMPL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} src)

# Find the Triangle library
# find_library(TRIANGLE_LIBRARY NAMES triangle PATHS ~/triangle NO_DEFAULT_PATH)

# Check if the Triangle library was found
# if (NOT TRIANGLE_LIBRARY)
#     message("Triangle library not found")
# endif()

# Add a file glob to include all cpp files in the src directory
file(GLOB SRC_FILES "src/*.cpp")

# Create a library from the source files in src/
add_library(helper_functions STATIC ${SRC_FILES})

add_executable(main ${SOURCE_FILE})

# if (TRIANGLE_FOUND)
#     target_link_libraries(main ${OMPL_LIBRARIES} ${TRIANGLE_LIBRARY} helper_functions)
# else()
#     target_link_libraries(main ${OMPL_LIBRARIES} helper_functions)
# endif()

target_link_libraries(main ${OMPL_LIBRARIES} ${OpenCV_LIBS} helper_functions)
