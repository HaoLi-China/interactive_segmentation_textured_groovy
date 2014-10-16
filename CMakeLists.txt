cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#include_directories("/usr/include/openni/")
#include_directories("~/ros_workspace/perception_new/pcl/include/pcl-1.7/")
# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_executable(find_poke_point src/find_poke_point.cpp)
rosbuild_add_executable (compute_icp_server src/compute_icp_server.cpp)
rosbuild_add_executable (transform_depth_image src/transform_depth_image.cpp)
rosbuild_add_executable (test_client src/test_client.cpp)
rosbuild_add_executable (c_track_features src/c_track_features.cpp)
rosbuild_add_executable (c_track_features_3d src/c_track_features_3d.cpp)
rosbuild_add_executable (c_track_features_with_grasping src/c_track_features_with_grasping.cpp)

rosbuild_add_executable(my_client src/my_client.cpp)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
