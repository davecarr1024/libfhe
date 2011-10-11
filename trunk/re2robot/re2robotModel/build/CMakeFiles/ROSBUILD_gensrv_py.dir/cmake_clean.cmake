FILE(REMOVE_RECURSE
  "../src/re2robotModel/msg"
  "../src/re2robotModel/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/re2robotModel/srv/__init__.py"
  "../src/re2robotModel/srv/_RemoveArbiter.py"
  "../src/re2robotModel/srv/_AddTransmission.py"
  "../src/re2robotModel/srv/_RemoveTransmission.py"
  "../src/re2robotModel/srv/_AddArbiter.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
