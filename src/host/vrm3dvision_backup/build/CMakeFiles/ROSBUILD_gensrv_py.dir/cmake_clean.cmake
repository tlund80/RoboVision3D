FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/vrm3dvision/msg"
  "../src/vrm3dvision/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/vrm3dvision/srv/__init__.py"
  "../src/vrm3dvision/srv/_setActiveCameras.py"
  "../src/vrm3dvision/srv/_createNewModel.py"
  "../src/vrm3dvision/srv/_startCamera.py"
  "../src/vrm3dvision/srv/_acquireSequenceOnRobot.py"
  "../src/vrm3dvision/srv/_setFrameRate.py"
  "../src/vrm3dvision/srv/_triggerCamera.py"
  "../src/vrm3dvision/srv/_setTriggerMode.py"
  "../src/vrm3dvision/srv/_setMode.py"
  "../src/vrm3dvision/srv/_setGain.py"
  "../src/vrm3dvision/srv/_setExposure.py"
  "../src/vrm3dvision/srv/_saveNextSequence.py"
  "../src/vrm3dvision/srv/_acquirePoseEstimate.py"
  "../src/vrm3dvision/srv/_stopCamera.py"
  "../src/vrm3dvision/srv/_computeBestExposure.py"
  "../src/vrm3dvision/srv/_setPattern.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
