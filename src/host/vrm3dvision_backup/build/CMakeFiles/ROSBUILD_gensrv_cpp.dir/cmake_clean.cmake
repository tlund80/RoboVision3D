FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/vrm3dvision/msg"
  "../src/vrm3dvision/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/vrm3dvision/setActiveCameras.h"
  "../srv_gen/cpp/include/vrm3dvision/createNewModel.h"
  "../srv_gen/cpp/include/vrm3dvision/startCamera.h"
  "../srv_gen/cpp/include/vrm3dvision/acquireSequenceOnRobot.h"
  "../srv_gen/cpp/include/vrm3dvision/setFrameRate.h"
  "../srv_gen/cpp/include/vrm3dvision/triggerCamera.h"
  "../srv_gen/cpp/include/vrm3dvision/setTriggerMode.h"
  "../srv_gen/cpp/include/vrm3dvision/setMode.h"
  "../srv_gen/cpp/include/vrm3dvision/setGain.h"
  "../srv_gen/cpp/include/vrm3dvision/setExposure.h"
  "../srv_gen/cpp/include/vrm3dvision/saveNextSequence.h"
  "../srv_gen/cpp/include/vrm3dvision/acquirePoseEstimate.h"
  "../srv_gen/cpp/include/vrm3dvision/stopCamera.h"
  "../srv_gen/cpp/include/vrm3dvision/computeBestExposure.h"
  "../srv_gen/cpp/include/vrm3dvision/setPattern.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
