#ifndef __VRM_GLOBAL_H__
#define __VRM_GLOBAL_H__

// Camera settings limits
#define VRM_EXPOSURE_MIN 0.1
#define VRM_EXPOSURE_MAX 500.0
#define VRM_GAIN_MIN 1
#define VRM_GAIN_MAX 255
#define VRM_FRAMERATE_MIN 0.02
#define VRM_FRAMERATE_MAX 500

// Mode limits
#define VRM_PATTERN_LEVELS_MIN 5
#define VRM_PATTERN_LEVELS_MAX 10

// Fixed defines - dont change
#define LOG_ERROR_LEVEL 1
#define LOG_WARNING_LEVEL 2
#define LOG_INFO_LEVEL 3

// User changeable defines
#define LOG_LEVEL LOG_INFO_LEVEL

#define LOG_TO_CONSOLE
#define LOG_TO_HOST
#define LOG_TO_FILE

#define VRM_COMMAND_SERVER_PORT "6000"
#define VRM_IMAGE_SERVER_PORT "6001"
#define VRM_LOG_SERVER_PORT "6002"

#endif //__VRM_GLOBAL_H__
