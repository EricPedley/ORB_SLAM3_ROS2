# Findlibpointmatcher.cmake
# This module finds the libpointmatcher package

# Adjust this path to where the headers are located in your project
find_path(POINTMATCHER_INCLUDE_DIR
    NAMES pointmatcher/PointMatcher.h
    HINTS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libpointmatcher/include
)

# Adjust this path to where the library is located in your project
find_library(POINTMATCHER_LIBRARY
    NAMES pointmatcher
    HINTS
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libpointmatcher/lib
)

# Check if the library and include directories were found
if(NOT POINTMATCHER_INCLUDE_DIR OR NOT POINTMATCHER_LIBRARY)
    message(FATAL_ERROR "libpointmatcher package not found")
endif()

# Store the results in variables
set(libpointmatcher_INCLUDE_DIRS ${POINTMATCHER_INCLUDE_DIR} PARENT_SCOPE)
set(libpointmatcher_LIBRARIES ${POINTMATCHER_LIBRARY} PARENT_SCOPE)
