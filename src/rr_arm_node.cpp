#include <iostream>
#include <rr_arm_ser.h>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/function.hpp>
#include <sensor_msgs/JointState.h>
// #include <trajectory_msgs/JointTrajectory.h>

std::string com = "/dev/ttyACM0";
RR_Arm rr_(com);

class rrArmTrajectoryServer
{
    public:
    rrArmTrajectoryServer(ros::NodeHandle& _nh):
    as_(_nh, "/arm_position_controller/follow_joint_trajectory", false)
    {
        as_.registerGoalCallback(boost::bind(&rrArmTrajectoryServer::executeCallback, this));
        as_.registerPreemptCallback(boost::bind(&rrArmTrajectoryServer::preemptedCallback, this));
        as_.start();
        ROS_INFO("FollowJointTrajectory action server started");
    };

    void executeCallback()
    {
        ROS_INFO("Preempt / execute → execute goal");
        auto goal = as_.acceptNewGoal();
        
        if(!goal)
        {
            ROS_INFO("Goal pointer is null");
            return;
        }
    };

    void preemptedCallback()
    {
        ROS_INFO("Preempt / cancel requested → stopping current motion");
    };

    private:
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rr_arm_node");
    ros::NodeHandle nh("~");
    rrArmTrajectoryServer server(nh);
    ros::Publisher joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        if(rr_.checkByte())
        {
            rr_.readJointState(rr_.byteArray, sizeof(rr_.byteArray));
            rr_.getJointState();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}