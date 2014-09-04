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

include(FindProtobuf.cmake)
include(FindZeroMQ.cmake)
rosbuild_add_boost_directories()

# Generate dynamic parameters
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

# include common header files shared with VRmagic D3 code
include_directories(../../common)
include_directories($ENV{HALCONROOT}/include)

# VRM Command Node
	rosbuild_add_executable(vrm_command_node 	src/vrm_command/vrm_command_node.cpp 
							src/vrm_command/vrm_command.cpp
							../../common/vrm_protocol/MsgHeaders.pb.cc)
											
	rosbuild_add_compile_flags(vrm_command_node -std=c++0x) # needed for the vrm_protocol
	
	target_link_libraries(vrm_command_node ${ZMQ_LIBRARIES} ${PROTOBUF_LIBRARIES})
# End of VRM Command Node

# VRM Image Proc Node
	rosbuild_add_executable(vrm_image_proc 	src/vrm_image_proc/vrm_image_proc_node.cpp 
						src/vrm_image_proc/vrm_image_proc.cpp
						src/vrm_image_proc/structured_light_reconstruction.cpp
						src/vrm_image_proc/occlusion_mask.cpp
						src/vrm_image_proc/visualization.cpp
						src/pose_estimation/pose_estimator.cpp
						src/pose_estimation/surface_model_estimator.cpp
						src/pose_estimation/object_model.cpp
						src/pose_estimation/alignment_prerejective.cpp
						../../common/vrm_protocol/MsgHeaders.pb.cc)
					
	rosbuild_add_compile_flags(vrm_image_proc -std=c++0x) # needed for the vrm_protocol
	target_link_libraries(vrm_image_proc ${ZMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so $ENV{HALCONROOT}/lib/$ENV{HALCONARCH}/libhalconcpp.so)
	rosbuild_link_boost(vrm_image_proc system filesystem)
	rosbuild_add_openmp_flags(vrm_image_proc)
# End of VRM Image Proc Node

#VRM Log Node
	rosbuild_add_executable(vrm_log_node 	src/vrm_log/vrm_log_node.cpp)			
	rosbuild_add_compile_flags(vrm_log_node -std=c++0x) # needed for the vrm_protocol
	target_link_libraries(vrm_log_node ${ZMQ_LIBRARIES})
# End of VRM Log Node

# zmq_image_publisher_node  Node
	rosbuild_add_executable(zmq_image_publisher_node 	src/zmq_image_publisher/zmq_image_publisher_node.cpp
								../../common/vrm_protocol/MsgHeaders.pb.cc)
	rosbuild_add_compile_flags(zmq_image_publisher_node -std=c++0x) # needed for the vrm_protocol
	target_link_libraries(zmq_image_publisher_node ${ZMQ_LIBRARIES} ${PROTOBUF_LIBRARIES})
# End of zmq_image_publisher_node

# pcl_pointcloud_viewer  Node
	rosbuild_add_executable(pcl_pointcloud_viewer 	src/pcl_pointcloud_viewer/pcl_pointcloud_viewer_node.cpp)
	target_link_libraries(pcl_pointcloud_viewer libvtkCommon.so libvtkFiltering.so libvtkRendering.so )
# End of pcl_pointcloud_viewer

# vrm_calibration  Node
	rosbuild_add_executable(vrm_calibration_node 	src/vrm_calibration/vrm_calibration_node.cpp
													src/vrm_calibration/vrm_calibration.cpp)
	rosbuild_add_openmp_flags(vrm_calibration_node)
# End of vrm_calibration Node

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
