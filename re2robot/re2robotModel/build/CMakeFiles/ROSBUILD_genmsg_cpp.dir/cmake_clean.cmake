FILE(REMOVE_RECURSE
  "../src/re2robotModel/msg"
  "../src/re2robotModel/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/re2robotModel/ModelConfig.h"
  "../msg_gen/cpp/include/re2robotModel/JointState.h"
  "../msg_gen/cpp/include/re2robotModel/TransmissionConfig.h"
  "../msg_gen/cpp/include/re2robotModel/ArbiterConfig.h"
  "../msg_gen/cpp/include/re2robotModel/JointCmd.h"
  "../msg_gen/cpp/include/re2robotModel/ControlArbiterState.h"
  "../msg_gen/cpp/include/re2robotModel/ControlArbiterCmd.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
