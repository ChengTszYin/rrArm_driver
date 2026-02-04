#include <rr_arm_ser.h>

RR_Arm::RR_Arm(const std::string& port_name): com(port_name)
{
    try
    {
        ser_ptr = new serial::Serial(port_name, 115200, serial::Timeout::simpleTimeout(1000));
        ser_ptr->setBytesize(serial::eightbits);
        ser_ptr->setParity(serial::parity_none);
        ser_ptr->setStopbits(serial::stopbits_one);
        ser_ptr->setFlowcontrol(serial::flowcontrol_none);
        if(ser_ptr->isOpen())
        {
            ROS_INFO("Port %s is opened successfully\n", port_name.c_str());
        }
    }

    catch(const std::exception& e)
    {
        ROS_ERROR("Serial error: %s", e.what());
        delete ser_ptr;
        ser_ptr = nullptr;
    } 
};

void RR_Arm::updateGoalTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
    auto points = traj.points;
    auto joint_names = traj.joint_names;
    size_t point_size = points.size();
    ROS_INFO("Receive total num_point: %li", point_size);
    for(size_t p = 0; p < point_size; ++p)
    {
        if(p==point_size-2)
        {
            auto point = points[p];
            if(point.positions.size() != NUM_JOINTS)
            {
                ROS_ERROR("Point %zu has wrong number of positions (%zu)", p, point.positions.size());
                continue;
            }
            ROS_INFO("At point %zu", p);
            std::copy(point.positions.begin(), point.positions.begin() + NUM_JOINTS, _setPosition);
            std::copy(point.velocities.begin(), point.velocities.begin() + NUM_JOINTS, _setVelocity);
            // ROS_INFO("_setPosition: %.3f %.3f %.3f %.3f %.3f %.3f", _setPosition[0], _setPosition[1], _setPosition[2], _setPosition[3], _setPosition[4], _setPosition[5]);
            // ROS_INFO("_setVelocity: %.3f %.3f %.3f %.3f %.3f %.3f", _setVelocity[0], _setVelocity[1], _setVelocity[2], _setVelocity[3], _setVelocity[4], _setVelocity[5]);
            for(auto& n: _setPosition)
            {
                n *= static_cast<float>(180.0 / M_PI);
            }
            driveSpeed(_setPosition, _setVelocity);
        }
    }
}

void RR_Arm::driveSpeed(const float (&angle)[6], const float (&velocity)[6])
{
    uint8_t buffer[48] = {0};
    memcpy(&buffer[0], angle, NUM_JOINTS * sizeof(float));
    memcpy(&buffer[24], velocity, NUM_JOINTS * sizeof(float));
    ser_ptr->write(buffer, sizeof(buffer));
}

int RR_Arm::checkByte()
{
    if(!ser_ptr->isOpen() || ser_ptr == nullptr)
    {
        return 0;
    }
    std::string rawByte = ser_ptr->read(ser_ptr->available());
    if(rawByte.size() != js_byte_size_) return 0;
    
    memcpy(byteArray, rawByte.data(), js_byte_size_);
    return 1;
}

void RR_Arm::readJointState(uint8_t* byteArray, int length)
{
    for(int i=0; i < 6; ++i)
    {
        uint8_t temp[4];
        memcpy(temp, byteArray + i*4, 4);
        for(auto& n:temp)
        {
            n *= M_PI / 180.0;
        }
        memcpy(&joint_state[i], temp, sizeof(float));
    }
}

float* RR_Arm::getJointState()
{
    return joint_state;
    // ROS_INFO("JointState: %.3f %.3f %.3f %.3f %.3f %.3f\n", joint_state[0], joint_state[1], joint_state[2], joint_state[3], joint_state[4], joint_state[5]);
}

uint8_t RR_Arm::checksum(uint8_t data[], int len) {
    int16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc = (crc + data[i]) & 0xFF;
    }
    return crc;
}

RR_Arm::~RR_Arm()
{
    ser_ptr->close();
    delete ser_ptr;
    ser_ptr = nullptr;
};