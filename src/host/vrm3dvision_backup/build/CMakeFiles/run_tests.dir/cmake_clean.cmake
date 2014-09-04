FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/vrm3dvision/msg"
  "../src/vrm3dvision/srv"
  "CMakeFiles/run_tests"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/run_tests.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
