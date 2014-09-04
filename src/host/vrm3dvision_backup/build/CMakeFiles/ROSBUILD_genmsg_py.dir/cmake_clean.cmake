FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/vrm3dvision/msg"
  "../src/vrm3dvision/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/vrm3dvision/msg/__init__.py"
  "../src/vrm3dvision/msg/_SurfaceModelParams.py"
  "../src/vrm3dvision/msg/_AlignmentPrerejectiveParams.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
