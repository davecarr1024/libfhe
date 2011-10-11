FILE(REMOVE_RECURSE
  "../src/re2robotModel/msg"
  "../src/re2robotModel/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/RemoveArbiter.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_RemoveArbiter.lisp"
  "../srv_gen/lisp/AddTransmission.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AddTransmission.lisp"
  "../srv_gen/lisp/RemoveTransmission.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_RemoveTransmission.lisp"
  "../srv_gen/lisp/AddArbiter.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AddArbiter.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
