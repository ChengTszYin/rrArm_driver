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
    for(size_t p = 0; p < traj.points.size(); ++p)
    {
        if(p)
        {
            ROS_INFO("At point %zu", p);
            auto pos = traj.points[p]; 
            for(size_t q = 0; q < pos.positions.size(); ++q)
            {
                m_pos[q] = pos.positions[q] * 180 / M_PI;
                ROS_INFO("Position[%zu] = %.3f", q, m_pos[q]);
            }
        }
    }
    driveSpeed(m_pos);
}

void RR_Arm::driveSpeed(const float (&angle)[6])
{
    uint8_t buffer[24];
    memcpy(buffer, angle, sizeof(angle));
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
        memcpy(&joint_state[i], temp, sizeof(float));
    }
}

void RR_Arm::getJointState()
{
    ROS_INFO("JointState: %.3f %.3f %.3f %.3f %.3f %.3f\n", joint_state[0], joint_state[1], joint_state[2], joint_state[3], joint_state[4], joint_state[5]);
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