#include<iostream>
#include <ros/ros.h>
#include "zlac8015_ros/zlac8015.h"
#include "zlac8015_ros/ros_wrapper.h"
#include<time.h>
int main(int argc ,char** argv)
{
    ros::init(argc ,argv , "base_motor_node");

    ZLAC8015_ROS_WRAPPER object("/dev/left_motor","/dev/right_motor",0x01);  //set usb port name

     return 0;
}
