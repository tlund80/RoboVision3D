FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/dense_reconstruction/DisparitySGBMConfig.h"
  "../docs/DisparitySGBMConfig.dox"
  "../docs/DisparitySGBMConfig-usage.dox"
  "../src/dense_reconstruction/cfg/DisparitySGBMConfig.py"
  "../docs/DisparitySGBMConfig.wikidoc"
  "../cfg/cpp/dense_reconstruction/DisparityConfig.h"
  "../docs/DisparityConfig.dox"
  "../docs/DisparityConfig-usage.dox"
  "../src/dense_reconstruction/cfg/DisparityConfig.py"
  "../docs/DisparityConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
