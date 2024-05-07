#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "assignment_ur/GetState.h"
#include "assignment_ur/JointSpaceMotion.h"
#include "assignment_ur/CartesianSpaceMotion.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_generator.hpp"
#include "actionlib/client/simple_action_client.h"
#include "assignment_ur/MoveHome.h"

#define PI 3.14159265359

class UR5_Service
{
public:
    UR5_Service(): move_home_client("/eff_joint_traj_controller/follow_joint_trajectory", true)
    {
        // Define the service
        get_state_sv            = nh.advertiseService("/ur5_custom_service/get_robot_state", &UR5_Service::getRobotState, this);
        move_joint_space_sv     = nh.advertiseService("/ur5_custom_service/move_joint_space", &UR5_Service::moveJointSpace, this);
        move_cartesian_space_sv = nh.advertiseService("/ur5_custom_service/move_cartesian_space", &UR5_Service::moveCartesianSpace, this);
        move_home_sv            = nh.advertiseService("/ur5_custom_service/move_home", &UR5_Service::moveHome, this);

        goal_joint_pub          = nh.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 1);

        ROS_INFO("UR5 Service has been started");
    }

    bool moveHome(assignment_ur::MoveHome::Request &request,
                       assignment_ur::MoveHome::Response &response)
    {
        auto joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh, ros::Duration(0.5));

        // create a goal
        control_msgs::FollowJointTrajectoryGoal goal;

        // set joint name 
        for (auto name : joint_state->name) {
            goal.trajectory.joint_names.push_back(name);
        }

        // set home position and velocity
        trajectory_msgs::JointTrajectoryPoint home_point;
        home_point.positions  = {0.0, -PI/2, 0.0, -PI/2, 0.0, 0.0};
        home_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        home_point.time_from_start = ros::Duration(3.0);
        goal.trajectory.points.push_back(home_point);

        // send goal
        move_home_client.waitForServer();
        move_home_client.sendGoal(goal);

        auto result = move_home_client.waitForResult();
        if (result) {
            ROS_INFO("ROBOT is in HOME position");
            return true;
        }
        else return false;
    }

    bool getRobotState(assignment_ur::GetState::Request &request,
                       assignment_ur::GetState::Response &response)
    {
        // wait for joint_states message
        auto msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh, ros::Duration(0.5));
        if (msg) {
            response.state = *msg;
            return true;
        } else {
            ROS_WARN("Timeout while waiting for JointState message");
            return false;
        }
    }

    bool moveJointSpace(assignment_ur::JointSpaceMotion::Request &request,
                        assignment_ur::JointSpaceMotion::Response &response)
    {
        // Step1: get current state 
        auto joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh, ros::Duration(0.5));
        auto start = std::move(request.joints1);
        auto end   = std::move(request.joints2);
        auto max_vel = request.joints_vel;
        auto max_acc = request.joints_acc;
        double time_step = 0.2;
        double time_from_start = 3.0; // time to move robot from current position to start position

        // Take current state as start if no start input
        if (start.size() == 0)  
        {
            ROS_INFO("Use current state as start point");
            start = std::move(joint_state->position);
            time_from_start = 0.0;
        }

        // Check valid condition
        if ((start.size() > 0 && start.size() != joint_state->name.size()) || end.size() != joint_state->name.size()) {
            ROS_ERROR("Check size of Start or End point");
            response.result = false;
            return false;
        }

        if (max_vel == 0 || max_acc == 0) {
            ROS_ERROR("Velocity and Acceleration should be > 0");
            response.result = false;
            return false;            
        }

        // Set parameters for traj-gen
        traj_generator.setParameters(start, end, max_vel, max_acc, time_step);

        // make a goal trajecotry 
        trajectory_msgs::JointTrajectory goal_traj;

        // set joint name 
        goal_traj.joint_names = std::move(joint_state->name);

        // set joint trajectory 
        traj_generator.generateJointSpaceTrajectory(goal_traj, time_from_start);

        // Publish trajectory
        goal_joint_pub.publish(goal_traj);
        
        response.result = true;
        return true;
    }

    bool moveCartesianSpace(assignment_ur::CartesianSpaceMotion::Request &request,
                        assignment_ur::CartesianSpaceMotion::Response &response)
    {
        
    }


private:
    ros::NodeHandle nh; // rosnode handle
    ros::ServiceServer move_home_sv;
    ros::ServiceServer get_state_sv; // service get robot state
    ros::ServiceServer move_joint_space_sv; 
    ros::ServiceServer move_cartesian_space_sv;
    ros::Publisher goal_joint_pub;

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move_home_client;

    tracjectory_generator::TrajectoryGenerator traj_generator; // trajectory_generator
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_server");
    UR5_Service ur5_server;
    ros::spin();
}