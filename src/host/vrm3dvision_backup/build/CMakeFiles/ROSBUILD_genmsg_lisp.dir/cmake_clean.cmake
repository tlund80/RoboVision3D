FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/vrm3dvision/msg"
  "../src/vrm3dvision/srv"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/SurfaceModelParams.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_SurfaceModelParams.lisp"
  "../msg_gen/lisp/AlignmentPrerejectiveParams.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_AlignmentPrerejectiveParams.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
