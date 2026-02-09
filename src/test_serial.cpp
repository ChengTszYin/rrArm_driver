#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <string>

std::string com = "/dev/ttyACM0";
uint32_t baud_rate = 9600;
serial::Serial ser(com, baud_rate, serial::Timeout::simpleTimeout(1000));
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "your_node_name");
    ros::NodeHandle nh;
    int timeout_ms = 1000;
    serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
    auto start_time = ros::Time::now();
    int i = 0;
    while(ros::ok())
    {
       if((ros::Time::now() - start_time).toSec() > (timeout_ms / 1000.0))
       {
            ROS_INFO("Iteration: %d", i);
            i++;
            start_time = ros::Time::now();
       }
    }
    
    ros::spin();

    return 0;
}