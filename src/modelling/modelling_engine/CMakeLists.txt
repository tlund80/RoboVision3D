cmake_minimum_required(VERSION 2.4.6)
project(modelling_engine)

set(CMAKE_BUILD_TYPE MinSizeRel)
#list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  structured_light_scanner
  simulated_kinect
  sensor_msgs
  pcl_ros
  message_generation
)

find_package(Boost REQUIRED)
find_package(VTK REQUIRED)
include_directories(${VTK_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   StartAcquisition.srv
   ClearModel.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robovision_communicator
  CATKIN_DEPENDS  std_msgs structured_light_scanner simulated_kinect sensor_msgs pcl_ros message_runtime
#  DEPENDS opencv
)

###########
## Build ##
###########

SET(DTI_COMMON_ROOT_DIR "$ENV{DTI_COMMON_ROOT}/DTI")
INCLUDE_DIRECTORIES(${DTI_COMMON_ROOT_DIR})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
)

## Declare a cpp library
#add_library(processor src/libdense_reconstruction/processor.cpp)

## Declare a cpp executable
add_executable(${PROJECT_NAME}
               src/main.cpp
               src/RosInterface.cpp
               src/SharedData.cpp
               src/ScanData.cpp
               src/RegistrationNode.cpp
               src/ReconstructPointCloud.cpp
	       src/tiv_types.cpp
	       src/rot.cpp
	       ${DTI_COMMON_ROOT_DIR}/Logger/DTILogger.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} 
		 ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME} 
			${catkin_LIBRARIES}
			${PCL_LIBRARIES}
			${Boost_LIBRARIES}
			${VTK_LIBRARIES}
			liblog4cxx.so
			)	
			
add_executable(${PROJECT_NAME}_ctrl
               src/modelling_engine_ctrl.cpp
	       ${DTI_COMMON_ROOT_DIR}/Logger/DTILogger.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}_ctrl 
		 ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}_ctrl 
			${catkin_LIBRARIES}
			${Boost_LIBRARIES}
			liblog4cxx.so
			)						

