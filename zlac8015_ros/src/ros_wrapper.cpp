#include "zlac8015_ros/ros_wrapper.h"




ZLAC8015_ROS_WRAPPER:: ZLAC8015_ROS_WRAPPER(std::string left_motor_port , std::string right_motor_port,uint8_t ID):_left_motor_port(left_motor_port),_right_motor_port(right_motor_port),_ID(ID)
{
    
    initialize_subscriber();
    ROS_DEBUG("!! INITIALIZED SUBSCRIBER !!");
    init_motor_params();
    ROS_DEBUG("!! INITIALIZED MOTOR PARAMS !!");

    publish_encoder_data();
    
}


void ZLAC8015_ROS_WRAPPER::initialize_subscriber()
{
    _vel_sub = _node.subscribe("/pgv/cmd_vel", 10 ,&ZLAC8015_ROS_WRAPPER::ros_callback,this);

    _motor_param_sub = _node.subscribe("/pgv/motor_parameter", 10 ,&ZLAC8015_ROS_WRAPPER::set_motor_parameters_callback,this);

    _encoder_left_pub = _node.advertise<std_msgs::Int64>("/left_wheel_encoder",1);
    _encoder_right_pub = _node.advertise<std_msgs::Int64>("/right_wheel_encoder",1); 

    


}

void ZLAC8015_ROS_WRAPPER::init_motor_params()
{
    _left_motor.begin(_left_motor_port,_ID);
    _right_motor.begin(_right_motor_port,_ID);
    
    _left_motor.enable();
    _right_motor.enable();
    sleep(1.0);
    
    _left_motor.set_vel_mode();
    _right_motor.set_vel_mode();
   
    sleep(1.0);
  

    _left_motor.set_acc_time(50);
   
    _right_motor.set_acc_time(50);
   sleep(1.0);
    _left_motor.set_decc_time(50);
   
    _right_motor.set_decc_time(50);
   sleep(1.0);
    _left_motor.max_speed(150);
    _right_motor.max_speed(150);
 sleep(1.0);

    
}

void ZLAC8015_ROS_WRAPPER::ros_callback(const geometry_msgs::Twist::ConstPtr & vel)
{
   
    ms_to_rpm(vel);
     _left_motor.set_rpm(_left_wheel_rpm);
    _right_motor.set_rpm(-_right_wheel_rpm);



}



void ZLAC8015_ROS_WRAPPER::set_motor_parameters_callback(const zlac8015_ros::MotorParameter::ConstPtr & param)
{
    
    if(param->type == "acc")
    {
      
        
        _left_motor.set_acc_time(param->value.data);
        sleep(1.0);
   
        _right_motor.set_acc_time(param->value.data);

        ROS_INFO("!! acceleration set !! : %d", param->value.data);
        
     

    }

    else if(param->type == "dec")
    {
        _left_motor.set_decc_time(param->value.data);
        sleep(1.0);
   
        _right_motor.set_decc_time(param->value.data);

        ROS_INFO("!! deacceleration set !! : %d", param->value.data);

    }

}



void ZLAC8015_ROS_WRAPPER::ms_to_rpm(const geometry_msgs::Twist::ConstPtr &vel)
{
    double vel_x = vel->linear.x;
    double rot_z = vel->angular.z;

    double right_wheel_speed = vel_x + (rot_z * _base_width) / 2;
    
    double left_wheel_speed = vel_x - (rot_z * _base_width) / 2;
    std::cout << left_wheel_speed << std::endl;
    

    _right_wheel_rpm =  ((right_wheel_speed) * 60.0) / (2 * 3.14 * _wheel_radius);
    _left_wheel_rpm = ((left_wheel_speed) * 60) / (2 * 3.14 * _wheel_radius);
 
}


void ZLAC8015_ROS_WRAPPER::publish_encoder_data()
{
   
   ros::Rate rate(100);
   while(ros::ok())
   {
        std_msgs::Int64 left_encoder , right_encoder;
        
        if(!_left_motor.read_motor())
        {
            left_encoder.data = _left_motor.get_position();

        }
        if(!_right_motor.read_motor())
        {
            right_encoder.data = _right_motor.get_position();

        }

        _encoder_left_pub.publish(left_encoder);
        _encoder_right_pub.publish(right_encoder);
        
        ros::spinOnce();
        rate.sleep();



    } 
}


ZLAC8015_ROS_WRAPPER::~ ZLAC8015_ROS_WRAPPER()
{
    _left_motor.disable();
    _right_motor.disable();


}


