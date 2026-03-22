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

void RR_Arm::BackHome()
{
    float home_position[NUM_JOINTS] = {0.00f, 0.00f, 0.00f, 0.00f, 0.00f, 0.00f};
    float home_velocity[NUM_JOINTS] = {10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f};
    short step = 1;
    driveSpeed(step, home_position, home_velocity);
}

bool RR_Arm::waitACK(int timeout_ms)
{
    auto start_time = ros::Time::now();
    uint8_t ack_byte;
    size_t byte_size;
    while((ros::Time::now() - start_time).toSec() < (timeout_ms / 1000.0))
    {
        byte_size = ser_ptr->read(&ack_byte, 1);
        if(byte_size == 1 && ack_byte == 'D')
        {
            return true;
        }
        ros::Duration(0.01).sleep();
    }
    return false;   
}

void RR_Arm::updateGoalTrajectory(const trajectory_msgs::JointTrajectory& traj)
{
    auto points = traj.points;
    auto joint_names = traj.joint_names;
    size_t point_size = points.size();
    // ROS_INFO("Receive total num_point: %li", point_size);
    for(size_t p = 0; p < point_size; ++p)
    {
        auto point = points[p];
        if(point.positions.size() != NUM_JOINTS)
        {
            ROS_ERROR("Point %zu has wrong number of positions (%zu)", p, point.positions.size());
            continue;
        }

        bool send_this_point = false;
        if (p == 0) {
            send_this_point = true;                    
        }
        else if (p == point_size-1) {
            send_this_point = true;                   
        }
        else if ((p % step_size) == 0) {
            send_this_point = true;                    
        }

        if (send_this_point)
        {
            ROS_INFO("Sending point %zu / %zu", p, point_size-1
            );

            std::copy(point.positions.begin(),   point.positions.begin()   + NUM_JOINTS, _setPosition);
            std::copy(point.velocities.begin(),  point.velocities.begin()  + NUM_JOINTS, _setVelocity);
            ROS_INFO("_setPosition: %.3f %.3f %.3f %.3f %.3f %.3f", _setPosition[0], _setPosition[1], _setPosition[2], _setPosition[3], _setPosition[4], _setPosition[5]);
            // ROS_INFO("_setVelocity: %.3f %.3f %.3f %.3f %.3f %.3f", _setVelocity[0], _setVelocity[1], _setVelocity[2], _setVelocity[3], _setVelocity[4], _setVelocity[5]);
            // Convert to degrees (assuming STM32 expects degrees)
            for (auto& n : _setPosition) {
                n *= 180.0f / static_cast<float>(M_PI);
            }
            short step = static_cast<short>(p);   // or use point_size - 1 - p if you prefer reverse numbering
            driveSpeed(step, _setPosition, _setVelocity);
        }
    }
}


bool RR_Arm::checkTrajectoryFinished(float* set_point, int len)
{
    float* joint_state = getJointState();
    for(int i = 0; i < len; ++i)
    {
        ROS_INFO("JointState_radian %i: %.3f, SetPoint: %.3f\n",i , joint_state[i], set_point[i]);
        if(abs(joint_state[i] - set_point[i]) > TOLERANCE)
        {
            return false;
        }
    }

    return true;
}

void RR_Arm::driveSpeed(short int &step, const float (&angle)[6], const float (&velocity)[6])
{
    uint8_t buffer[50] = {0};
    buffer[0] = static_cast<uint8_t>(step & 0xFF);          // LSB
    buffer[1] = static_cast<uint8_t>((step >> 8) & 0xFF);
    memcpy(&buffer[2], angle, NUM_JOINTS * sizeof(float));
    memcpy(&buffer[26], velocity, NUM_JOINTS * sizeof(float));
    ser_ptr->write(buffer, sizeof(buffer));
    ros::Duration(0.08).sleep();    //minimum time for stm32 to execute each segment
}

int RR_Arm::checkByte()
{
    if(!ser_ptr->isOpen() || ser_ptr == nullptr)
    {
        return 0;
    }
    std::string rawByte = ser_ptr->read(ser_ptr->available());
    if(rawByte.size() == js_byte_size_ && checksum(rawByte, js_byte_size_))
    {
        memcpy(byteArray, rawByte.data(), js_byte_size_);
        return 1;
    }
    else
    {
        ROS_ERROR("js_byte_size_ is %li, expecting %i", rawByte.size(), js_byte_size_);
        return 0;
    }
}

void RR_Arm::readJointState(uint8_t* byteArray, int length)
{
    for(int i = 0; i < 6; ++i)
    {
        memcpy(&joint_state[i], byteArray + i*4, 4);

        joint_state[i] *= (M_PI / 180.0f);
    }
}

float* RR_Arm::getJointState()
{
    return joint_state;
    // ROS_INFO("JointState: %.3f %.3f %.3f %.3f %.3f %.3f\n", joint_state[0], joint_state[1], joint_state[2], joint_state[3], joint_state[4], joint_state[5]);
}

float* RR_Arm::getTrajectoryFinalPoint(const trajectory_msgs::JointTrajectory& traj)
{
    static float _setPosition[NUM_JOINTS] = {0.0f};
    if(traj.points.empty())
    {
        ROS_ERROR("Trajectory has no points");
        return _setPosition;
    }
    auto final_pose = traj.points.back();
    if(final_pose.positions.size() != NUM_JOINTS)
    {
        ROS_ERROR("Final pose has wrong number of positions (%zu)", final_pose.positions.size());
        return _setPosition;
    }
    std::copy(final_pose.positions.begin(), final_pose.positions.begin() + NUM_JOINTS, _setPosition);
    return _setPosition;
}   

uint8_t RR_Arm::checksum(std::string& data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += static_cast<uint8_t>(data[i]);
    }
    return sum;
}

RR_Arm::~RR_Arm()
{
    ser_ptr->close();
    delete ser_ptr;
    ser_ptr = nullptr;
};