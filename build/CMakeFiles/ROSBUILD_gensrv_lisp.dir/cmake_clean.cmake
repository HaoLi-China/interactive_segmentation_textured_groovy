FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/interactive_segmentation_textured/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/depthImage.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_depthImage.lisp"
  "../srv_gen/lisp/estimateRigid.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_estimateRigid.lisp"
  "../srv_gen/lisp/computeICP.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_computeICP.lisp"
  "../srv_gen/lisp/cornerPokePoseFind.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_cornerPokePoseFind.lisp"
  "../srv_gen/lisp/cornerFind.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_cornerFind.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
