cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

# Dynamic reconfigure support
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# Nodelet library
rosbuild_add_library(processor src/libdense_reconstruction/processor.cpp)
rosbuild_add_library(disparity src/nodelets/disparity.cpp)
rosbuild_add_library(disparitySGBM src/nodelets/disparitySGBM.cpp)
rosbuild_add_library(point_cloud2 src/nodelets/point_cloud2.cpp)
rosbuild_add_library(point_cloud src/nodelets/point_cloud.cpp)

# Standalone node
rosbuild_add_executable(dense_reconstruction src/dense_reconstruction.cpp)
target_link_libraries(dense_reconstruction processor disparity point_cloud2 point_cloud disparitySGBM)
SET_TARGET_PROPERTIES(dense_reconstruction PROPERTIES OUTPUT_NAME dense_reconstruction)
