# Global path definitions for all demos

#Specify target architecture here or give as command line option
# allowed values Linux_x86, Linux_x64, D3

#TARGET_ARCH=

# Path to your installation of the VRmagic SDK 
# for Linux_x86/x64 and D3: set SDK_VERSION to version you want to build against 
SDK_VERSION = 4.1.0
VRM_DIR = /opt/vrmagic/sdk-$(SDK_VERSION)

######################## Defines ######################
MKDIR = mkdir -p
RMDIR = rm -r -f
DEL_FILE = rm -f

############### DO NOT EDIT BELOW THIS LINE ###########

# Check Target Architecture

ALLOWED_ARCHS = Linux_x86 Linux_x64 D3

ifndef TARGET_ARCH
  TARGET_ARCH = D3
  $(info Target architecture not specified - Selecting D3)
endif

_TARGET_ARCH := $(filter $(TARGET_ARCH),$(ALLOWED_ARCHS))

$(info Building for $(_TARGET_ARCH))

ifndef _TARGET_ARCH
  $(error Variable TARGET_ARCH not set correctly! Possible target architectures: $(ALLOWED_ARCHS))
endif

## Linux_x86
VRMSDK_INSTALL_DIR_Linux_x86 = $(VRM_DIR)/x86/development_kit
CROSS_COMPILE_PREFIX_Linux_x86 =

VRM_INCPATH_Linux_x86 = -I$(VRMSDK_INSTALL_DIR_Linux_x86)/include \
-I/opt/ros/$(ROS_DISTRO)/include ## OpenCV (From ROS)

VRM_LIBPATH_Linux_x86 = -L$(VRMSDK_INSTALL_DIR_Linux_x86)/lib \
-L/opt/ros/$(ROS_DISTRO)/lib ## OpenCV (From ROS)
VRM_LFLAGS_Linux_x86 =

## Linux_x64
VRMSDK_INSTALL_DIR_Linux_x64 = $(VRM_DIR)/x64/development_kit
CROSS_COMPILE_PREFIX_Linux_x64 =
VRM_INCPATH_Linux_x64 = -I$(VRMSDK_INSTALL_DIR_Linux_x64)/include \
-I/usr/local/include \
-I../../common \
-I/opt/ros/$(ROS_DISTRO)/include ## OpenCV (From ROS)

VRM_LIBPATH_Linux_x64 = -L$(VRMSDK_INSTALL_DIR_Linux_x64)/lib/ \
-L/usr/local/lib \
-L/opt/ros/$(ROS_DISTRO)/lib ## OpenCV (From ROS)

VRM_LFLAGS_Linux_x64 =

## D3
CROSS_COMPILE_PREFIX_D3 = arm-linux-gnueabihf-
VRMSDK_INSTALL_DIR_D3 = $(VRM_DIR)/D3/development_kit
VRM_INCPATH_D3 = -I$(VRMSDK_INSTALL_DIR_D3)/include \
-I/usr/arm-linux-gnueabihf/include \
-I/usr/arm-linux-gnueabihf/vrmagic/usr/include \
-I../../common \
-I/usr/arm-linux-gnueabihf/opencv/include ## OpenCV

VRM_LIBPATH_D3 = -L$(VRMSDK_INSTALL_DIR_D3)/lib \
-L/usr/arm-linux-gnueabihf/lib \
-L/usr/arm-linux-gnueabihf/vrmagic/lib \
-L/usr/arm-linux-gnueabihf/vrmagic/lib/arm-linux-gnueabihf \
-L/usr/arm-linux-gnueabihf/vrmagic/usr/lib \
-L/usr/arm-linux-gnueabihf/vrmagic/usr/lib/arm-linux-gnueabihf \
-L/usr/arm-linux-gnueabihf/opencv/lib ## OpenCV

VRM_LFLAGS_D3 = -Wl,--rpath-link=$(VRMSDK_INSTALL_DIR_D3)/lib:/usr/arm-linux-gnueabihf/lib:/usr/arm-linux-gnueabihf/vrmagic/lib/:/usr/arm-linux-gnueabihf/vrmagic/lib/arm-linux-gnueabihf:/usr/arm-linux-gnueabihf/vrmagic/usr/lib:/usr/arm-linux-gnueabihf/vrmagic/usr/lib/arm-linux-gnueabihf

################ Put things together ####################

CROSS_COMPILE_PREFIX = ${CROSS_COMPILE_PREFIX_${_TARGET_ARCH}}
VRMSDK_INSTALL_DIR = ${VRMSDK_INSTALL_DIR_${_TARGET_ARCH}}
VRM_INCPATH          = ${VRM_INCPATH_${_TARGET_ARCH}}
VRM_LIBPATH          = ${VRM_LIBPATH_${_TARGET_ARCH}}
VRM_LFLAGS           = ${VRM_LFLAGS_${_TARGET_ARCH}}


