#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "assignment_ur/GetState.h"
#include "assignment_ur/JointSpaceMotion.h"
#include "assignment_ur/CartesianSpaceMotion.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

class UR5_Service
{
public:
    UR5_Service()
    {
        // Define the service
        get_state_sv            = nh.advertiseService("/ur5_custom_service/get_robot_state", &UR5_Service::getRobotState, this);
        move_joint_space_sv     = nh.advertiseService("/ur5_custom_service/move_joint_space", &UR5_Service::moveJointSpace, this);
        move_cartesian_space_sv = nh.advertiseService("/ur5_custom_service/move_cartesian_space", &UR5_Service::moveCartesianSpace, this);
        goal_joint_pub          = nh.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 1);

        ROS_INFO("UR5 Service has been started");
    }

    bool getRobotState(assignment_ur::GetState::Request &request,
                       assignment_ur::GetState::Response &response)
    {
        // wait for 
        auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh, ros::Duration(1.0));
        if (msg)
        {
            response.state = *msg;
            return true;
        } 
        else
        {
            ROS_WARN("Timeout while waiting for JointState message");
            return false;
        }
    }

    bool moveJointSpace(assignment_ur::JointSpaceMotion::Request &request,
                        assignment_ur::JointSpaceMotion::Response &response)
    {
        
    }

    bool moveCartesianSpace(assignment_ur::CartesianSpaceMotion::Request &request,
                        assignment_ur::CartesianSpaceMotion::Response &response)
    {
        
    }


private:
    ros::NodeHandle nh; // rosnode handle
    ros::ServiceServer get_state_sv; // service get robot state
    ros::ServiceServer move_joint_space_sv; 
    ros::ServiceServer move_cartesian_space_sv;
    ros::Publisher goal_joint_pub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_server");
    UR5_Service ur5_server;
    ros::spin();
}