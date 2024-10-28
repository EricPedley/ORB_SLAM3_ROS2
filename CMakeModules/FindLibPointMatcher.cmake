# # FindLibPointMatcher.cmake
# # This module finds the libpointmatcher package
#
# # Specify the required dependencies
# find_dependency(libnabo REQUIRED)
# find_dependency(yaml-cpp REQUIRED)
# find_package(Boost COMPONENTS thread filesystem system program_options date_time REQUIRED)
# if (Boost_MINOR_VERSION GREATER 47)
#   find_package(Boost COMPONENTS thread filesystem system_program_options date_time chrono REQUIRED)
# endif()
#
# # Define the paths
# get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)
#
# # Set the include directories
# set(libpointmatcher_INCLUDE_DIRS
#     "${PACKAGE_PREFIX_DIR}/include"
#     "/usr/local/include/eigen3"
#     CACHE PATH "Include directories for libpointmatcher")
#
# # Set the libraries to link against
# set(libpointmatcher_LIBRARIES
#     "$<TARGET_FILE:pointmatcher>"
#     "yaml-cpp"
#     "libnabo::nabo"
#     "Threads::Threads"
#     "Boost::thread"
#     "Boost::filesystem"
#     "Boost::system"
#     "Boost::program_options"
#     "Boost::date_time"
#     "Boost::chrono"
#     CACHE STRING "Libraries to link against for libpointmatcher")
#
# # Check if include directories and libraries exist
# if(NOT EXISTS "${libpointmatcher_INCLUDE_DIRS}")
#     message(FATAL_ERROR "Include directories for libpointmatcher do not exist!")
# endif()
#
# if(NOT EXISTS "${libpointmatcher_LIBRARIES}")
#     message(FATAL_ERROR "Libraries for libpointmatcher do not exist!")
# endif()
#
# # Set the results in cache for use in the main project
# mark_as_advanced(libpointmatcher_INCLUDE_DIRS libpointmatcher_LIBRARIES)
#
#
#
#
#
#
# # # Findlibpointmatcher.cmake
# # # This module finds the libpointmatcher package
# #
# # # Adjust this path to where the headers are located in your project
# # find_path(POINTMATCHER_INCLUDE_DIR
# #     NAMES pointmatcher/PointMatcher.h
# #     HINTS
# #     ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libpointmatcher/pointmatcher
# # )
# #
# # # Adjust this path to where the library is located in your project
# # find_library(POINTMATCHER_LIBRARY
# #     NAMES pointmatcher
# #     HINTS
# #     ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libpointmatcher/pointmatcher
# # )
# #
# # # Check if the library and include directories were found
# # if(NOT POINTMATCHER_INCLUDE_DIR OR NOT POINTMATCHER_LIBRARY)
# #     message(FATAL_ERROR "libpointmatcher package not found")
# # endif()
# #
# # # Store the results in variables
# # set(libpointmatcher_INCLUDE_DIRS ${POINTMATCHER_INCLUDE_DIR} PARENT_SCOPE)
# # set(libpointmatcher_LIBRARIES ${POINTMATCHER_LIBRARY} PARENT_SCOPE)
