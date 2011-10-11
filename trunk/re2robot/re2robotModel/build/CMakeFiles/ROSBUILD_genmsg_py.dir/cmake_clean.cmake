FILE(REMOVE_RECURSE
  "../src/re2robotModel/msg"
  "../src/re2robotModel/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/re2robotModel/msg/__init__.py"
  "../src/re2robotModel/msg/_ModelConfig.py"
  "../src/re2robotModel/msg/_JointState.py"
  "../src/re2robotModel/msg/_TransmissionConfig.py"
  "../src/re2robotModel/msg/_ArbiterConfig.py"
  "../src/re2robotModel/msg/_JointCmd.py"
  "../src/re2robotModel/msg/_ControlArbiterState.py"
  "../src/re2robotModel/msg/_ControlArbiterCmd.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
