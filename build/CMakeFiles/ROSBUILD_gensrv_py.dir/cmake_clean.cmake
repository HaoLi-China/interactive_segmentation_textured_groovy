FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/interactive_segmentation_textured/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/interactive_segmentation_textured/srv/__init__.py"
  "../src/interactive_segmentation_textured/srv/_cornerPokePoseFind.py"
  "../src/interactive_segmentation_textured/srv/_estimateRigid.py"
  "../src/interactive_segmentation_textured/srv/_depthImage.py"
  "../src/interactive_segmentation_textured/srv/_cornerFind.py"
  "../src/interactive_segmentation_textured/srv/_computeICP.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
