#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "std_msgs/String.h"
#include <sstream>

#include <interactive_segmentation_textured/cornerPokePoseFind.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "my_client");

    ros::NodeHandle n;

printf("11111111111111\n");

    ros::ServiceClient client_poke = n.serviceClient<interactive_segmentation_textured::cornerPokePoseFind>("findPokePose");
printf("22222222222222\n");
    interactive_segmentation_textured::cornerPokePoseFind srv_poke;
printf("33333333333333\n");

    if (client_poke.call(srv_poke))
    {
printf("44444444444444\n");
    } 

    return 0;
}
