FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/interactive_segmentation_textured/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/interactive_segmentation_textured/cornerPokePoseFind.h"
  "../srv_gen/cpp/include/interactive_segmentation_textured/estimateRigid.h"
  "../srv_gen/cpp/include/interactive_segmentation_textured/depthImage.h"
  "../srv_gen/cpp/include/interactive_segmentation_textured/cornerFind.h"
  "../srv_gen/cpp/include/interactive_segmentation_textured/computeICP.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
