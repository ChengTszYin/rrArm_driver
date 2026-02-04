#ifndef RM_ARM_SER_H
#define RM_ARM_SER_H
#include <iostream>
#include <serial/serial.h>
#include <string>
#include <chrono>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cmath>

#define js_byte_size_ 24
constexpr size_t NUM_JOINTS = 6;

struct rArm
{
    
};

class RR_Arm
{
    public:
    RR_Arm(const std::string& port_name);
    void updateGoalTrajectory(const trajectory_msgs::JointTrajectory& traj);
    void driveSpeed(const float (&velocity)[6], const float (&angle)[6]);
    int checkByte();
    void readJointState(uint8_t* byteArray, int length);
    uint8_t checksum(uint8_t data[], int len);
    uint8_t byteArray[js_byte_size_];
    float* getJointState();
    ~RR_Arm();

    private:
    std::string com;
    serial::Serial* ser_ptr = nullptr;
    float r_pos[6];
    float joint_state[6];
    float _setPosition[NUM_JOINTS];
    float _setVelocity[NUM_JOINTS];
};

#endif