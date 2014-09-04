FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/vrm3dvision/msg"
  "../src/vrm3dvision/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/vrm3dvision/SurfaceModelParams.h"
  "../msg_gen/cpp/include/vrm3dvision/AlignmentPrerejectiveParams.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
