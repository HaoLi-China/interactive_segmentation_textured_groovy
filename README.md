interactive_segmentation_textured_groovy
========================================

complie the interactive_segmentation_textured package from Bosch (bosch-ros-pkg) on ROS Groovy

运行步骤：
1.roslaunch gazebo_worlds empty_world_no_x.launch
2.roslaunch pr2_gazebo pr2.launch
3.rosrun openni_launch openni.launch
4.rosrun interactive_segmentation_textured_groovy find_corners.py
5.rosrun interactive_segmentation_textured_groovy find_poke_point
6.rosrun interactive_segmentation_textured_groovy my_clien
