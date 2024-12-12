#ifndef _ZLAC8015_ROS_WRAPPER_H_
#define _ZLAC8015_ROS_WRAPPER_H_
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include "zlac8015_ros/zlac8015.h"
#include <std_msgs/Int64.h>
#include "zlac8015_ros/MotorParameter.h"
#include <string>



class ZLAC8015_ROS_WRAPPER
{

    public:

        

        ZLAC8015_ROS_WRAPPER(std::string left_motor_port , std::string right_motor_port,uint8_t ID);

        void ms_to_rpm(const geometry_msgs::Twist::ConstPtr &vel);
    
        void initialize_subscriber();

        void init_motor_params();
    
        void ros_callback(const geometry_msgs::Twist::ConstPtr & vel);

        void set_motor_parameters_callback(const zlac8015_ros::MotorParameter::ConstPtr & param);

        void publish_encoder_data();

        ~ ZLAC8015_ROS_WRAPPER();

    private:
        ros::NodeHandle _node;
        PGV::ZLAC _right_motor;
        PGV::ZLAC _left_motor;
        
        std::string _left_motor_port;
        std::string _right_motor_port;
        uint8_t _ID;
        ros::Subscriber _vel_sub;
        ros::Subscriber _motor_param_sub;
        ros::Publisher _encoder_left_pub;
        ros::Publisher _encoder_right_pub;

        double _base_width = 0.55977;
        double _wheel_radius = 0.0825;
        double _left_wheel_rpm = 0;
        double _right_wheel_rpm = 0;
        double _gear_reduction = 20;



};
#endif
