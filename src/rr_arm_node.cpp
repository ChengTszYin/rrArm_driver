#include <iostream>
#include <rr_arm_ser.h>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <boost/function.hpp> 
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rArm/callRobot.h>

#define TIMEOUT_DURATION 3.0

std::string com = "/dev/ttyACM0";
RR_Arm rr_(com);

class rrArmTrajectoryServer
{
    public:
    rrArmTrajectoryServer(ros::NodeHandle& _nh):
    as_(_nh, "/rArm_controller/follow_joint_trajectory", false)
    {
        as_.registerGoalCallback(boost::bind(&rrArmTrajectoryServer::executeCallback, this));
        as_.registerPreemptCallback(boost::bind(&rrArmTrajectoryServer::preemptedCallback, this));
        as_.start();
        ROS_INFO("FollowJointTrajectory action server started");
    };

    void executeCallback()
    {
        ROS_INFO("execute goal");
        auto goal_ptr = as_.acceptNewGoal();
        if(!goal_ptr || goal_ptr->trajectory.points.empty())
        {
            ROS_INFO("Goal pointer is null");
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = result.INVALID_GOAL;
            as_.setAborted(result); 
            return;
        }

        active_goal_       = true;
        preempt_requested_ = false;
        executed_          = false;

        ros::Time start_time = ros::Time::now();    // record the starting time of trajectory execution
        auto traject = goal_ptr -> trajectory;
        float* final_pose_ptr = rr_.getTrajectoryFinalPoint(traject);
        rr_.updateGoalTrajectory(traject);

        while(ros::ok() && active_goal_)
        {
            if(rr_.checkByte())
            {
                ROS_INFO("read data");
                rr_.readJointState(rr_.byteArray, sizeof(rr_.byteArray));
            }

            if (preempt_requested_)
            {
                ROS_INFO("Goal preempted");
                control_msgs::FollowJointTrajectoryResult result;
                result.error_code = result.INVALID_GOAL;
                as_.setPreempted(result);
                active_goal_ = false;
                return;
            }

            if(rr_.checkTrajectoryFinished(final_pose_ptr, 6))
            {
                ROS_INFO("Trajectory execution finished successfully");
                executed_ = true;
                control_msgs::FollowJointTrajectoryResult result;
                result.error_code = result.SUCCESSFUL;
                as_.setSucceeded(result);
                active_goal_ = false;
                break;
            }

            if((ros::Time::now() - start_time) > ros::Duration(TIMEOUT_DURATION))   //
            {
                ROS_INFO("Trajectory execution timed out");
                control_msgs::FollowJointTrajectoryResult result;
                result.error_code = result.OLD_HEADER_TIMESTAMP;
                as_.setAborted(result);
                active_goal_ = false;
                executed_ = false;
                break;
            }
            ros::Duration(0.02).sleep();
            ros::spinOnce();
        }
    };

    void preemptedCallback()
    {
        ROS_INFO("Preempt requested");
        preempt_requested_ = true;
    };

    private:
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
    bool active_goal_       = false;
    bool preempt_requested_ = false;
    bool executed_          = false;
    bool robotReady_;
};

bool callPoses(rArm::callRobot::Request &req, rArm::callRobot::Response &res)
{
    float angle[6] = {0.0f};
    float angle_rad[6] = {0.0f};
    float velocity[6] = {10.0f};
    short int step = 1;
    for(int i = 0; i < 6; ++i)
    {
        angle[i] = static_cast<float>(req.poses[i]);
        angle_rad[i] = angle[i] * (M_PI / 180.0f);
    }
    ros::Time start_time = ros::Time::now();
    rr_.driveSpeed(step, angle, velocity);
    while(ros::ok())
    {
        if(rr_.checkByte())
        {
            rr_.readJointState(rr_.byteArray, sizeof(rr_.byteArray));
        }

        if(rr_.checkTrajectoryFinished(angle_rad, 6))
        {
            ROS_INFO("Trajectory execution finished successfully");
            res.result = true;
            break;
        }

        if((ros::Time::now() - start_time) > ros::Duration(TIMEOUT_DURATION))   //
        {
            ROS_INFO("Trajectory execution timed out");
            res.result = false;
            break;
        }
        ros::Duration(0.02).sleep();
        ros::spinOnce();
    }
    return res.result;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "rr_arm_node");
    ros::Time::init();
    ros::NodeHandle nh;
    rrArmTrajectoryServer server(nh);
    ros::Publisher joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);
    ros::ServiceServer pose_service = nh.advertiseService("call_robot_poses", callPoses);
    ros::Rate loop_rate(100);
    rr_.BackHome();
    ROS_INFO("Joint state initialized\n");
    while(ros::ok())
    {
        if(rr_.checkByte())
        {
            rr_.readJointState(rr_.byteArray, sizeof(rr_.byteArray));
            float* _joint_state = rr_.getJointState();
            sensor_msgs::JointState js;
            js.header.stamp = ros::Time::now();
            js.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
            js.position.assign(_joint_state, _joint_state + 6);
            joint_state_pub_.publish(js);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}