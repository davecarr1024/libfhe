FILE(REMOVE_RECURSE
  "../src/re2robotModel/msg"
  "../src/re2robotModel/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/re2robotModel/RemoveArbiter.h"
  "../srv_gen/cpp/include/re2robotModel/AddTransmission.h"
  "../srv_gen/cpp/include/re2robotModel/RemoveTransmission.h"
  "../srv_gen/cpp/include/re2robotModel/AddArbiter.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
