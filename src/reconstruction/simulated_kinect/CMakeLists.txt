cmake_minimum_required(VERSION 2.4.6)
project(simulated_kinect)

set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  robot_msgs
  tf
  actionlib
  actionlib_msgs
)

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
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   Scan.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs   
   robot_msgs   
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
#  INCLUDE_DIRS include
#  LIBRARIES robovision_communicator
  CATKIN_DEPENDS pcl_ros roscpp robot_msgs sensor_msgs std_msgs robot_msgs tf actionlib actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(RW_ROOT_DIR "$ENV{DTI_COMMON_ROOT}/RobWork/RW_new")
SET(RW_ROOT "${RW_ROOT_DIR}/RobWork")

SET(RobWorkStudio_DIR "${RW_ROOT}/RobWorkStudio/cmake") 
FIND_PACKAGE(RobWorkStudio REQUIRED)

SET(RobWork_DIR "${RW_ROOT}/RobWork/cmake") 
FIND_PACKAGE(RobWork REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ROBWORK_INCLUDE_DIRS}
  ${ROBWORKSTUDIO_INCLUDE_DIRS}
  $ENV{DTI_COMMON_ROOT}
  $ENV{DTI_COMMON_ROOT}/DTI
)
LINK_DIRECTORIES( ${ROBWORKSTUDIO_LIBRARY_DIRS} )

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# if we want to use ui files add them here
SET(UIS_FILES src/SensorsPlugin.ui )
IF(RWS_USE_QT5)
   RWS_QT5_WRAP_UI(UIS_OUT_H ${UIS_FILES})
ELSE()
   RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
ENDIF()
    
# Standard cpp files to compile:
#SET(SrcFiles src/ROS_Sensor.cpp src/SensorView.cpp)
SET(MocHeaderFiles 
    include/simulated_kinect/ROS_Sensor.hpp 
    include/simulated_kinect/SensorView.hpp
    include/simulated_kinect/RosInterface.h)

#set_source_files_properties(${SrcFiles} PROPERTIES OBJECT_DEPENDS "${UIS_OUT_H}")
SET(MocSrcFiles)
IF(RWS_USE_QT5)
  QT5_WRAP_CPP(MocSrcFiles ${MocHeaderFiles})
  # Rcc the files:
  QT5_ADD_RESOURCES(RccSrcFiles src/resources.qrc)
  ELSE()
  # Header files to moc and compile:
  QT4_WRAP_CPP(MocSrcFiles ${MocHeaderFiles})
  # Rcc the files:
  QT4_ADD_RESOURCES(RccSrcFiles src/resources.qrc)
ENDIF()

#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
 
#message(${PCL_LIBRARIES})
#common commands for building c++ executables and libraries
add_library(ROS_Sensor
            src/ROS_Sensor.cpp
	    src/SensorView.cpp
            src/RosInterface.cpp
            ${UIS_OUT_H}
            ${MocSrcFiles}  
            ${RccSrcFiles})

add_dependencies(ROS_Sensor ${PROJECT_NAME}_generate_messages_cpp)	     
target_link_libraries(ROS_Sensor ${catkin_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES}  ${PCL_LIBRARIES})

#add_executable(ROS_Sensor_Client src/test_client.cpp)
#target_link_libraries(ROS_Sensor_Client ${catkin_LIBRARIES})
#add_executable(actionClient_test src/ScanActionClient.cpp)
#target_link_libraries(actionClient_test ${catkin_LIBRARIES})

