cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/test)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/neighbours.cpp src/frontier_navigation.cpp src/qualities.cpp src/search.cpp
src/helpers.cpp src/types.h src/publishers.cpp src/filter.cpp src/constants.h src/map_operations.cpp)
rosbuild_add_library(${PROJECT_NAME}_test test/main.cpp test/test.cpp src/helpers.cpp src/map_operations.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(frontiers src/main.cpp )
target_link_libraries(frontiers ${PROJECT_NAME})
rosbuild_add_executable(frontiers_test test/main.cpp)
target_link_libraries(frontiers_test ${PROJECT_NAME}_test)
