FILE(REMOVE_RECURSE
  "../src/re2robotDriver/msg"
  "../src/re2robotDriver/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/re2robotDriver/msg/__init__.py"
  "../src/re2robotDriver/msg/_DriveCmd.py"
  "../src/re2robotDriver/msg/_DriverConfig.py"
  "../src/re2robotDriver/msg/_DriveState.py"
  "../src/re2robotDriver/msg/_DriveConfig.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
