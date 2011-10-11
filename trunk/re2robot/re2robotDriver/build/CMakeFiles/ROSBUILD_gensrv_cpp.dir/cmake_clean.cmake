FILE(REMOVE_RECURSE
  "../src/re2robotDriver/msg"
  "../src/re2robotDriver/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/re2robotDriver/AddDrive.h"
  "../srv_gen/cpp/include/re2robotDriver/RemoveDrive.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
