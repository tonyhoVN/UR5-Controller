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
#include "ros/package.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "tf/transform_listener.h"

#define PI 3.14

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

        // traj_gen
        traj_generator = std::make_shared<trajectory_generator::TrajectoryGenerator>();

        // Setup IK Solver 
        std::string urdf_path = ros::package::getPath("assignment_ur");
        if(urdf_path.empty()) {
            ROS_ERROR("assignment_ur package path was not found");
            return;
        }
        urdf_path += "/urdf/ur5_robot.urdf";
        ik_solver =  std::make_shared<trajectory_generator::IKSolver>(urdf_path, num_joint, base_link, end_effector);

        ROS_INFO("UR5 Service has been started");
    }

    bool moveHome(assignment_ur::MoveHome::Request &request,
                       assignment_ur::MoveHome::Response &response)
    {
        auto joint_state = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/eff_joint_traj_controller/state", nh, ros::Duration(1.0));

        // create a goal
        control_msgs::FollowJointTrajectoryGoal goal;

        // set joint name 
        goal.trajectory.joint_names = std::move(joint_state->joint_names);

        // set home position and velocity
        trajectory_msgs::JointTrajectoryPoint home_point;
        home_point.positions  = {0.0, 0.0, PI/2, 0.0, -PI/2, 0.0};
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
        // get joint position
        auto joint_state = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/eff_joint_traj_controller/state", nh, ros::Duration(1.0));
        if (joint_state) {
            response.joints_name = joint_state->joint_names;
            response.posj = joint_state->actual.positions;
        } else {
            ROS_WARN("Timeout while waiting for JointState message");
            return false;
        }

        // get cartesian position
        tf::TransformListener tf_;
        tf_.waitForTransform(base_link, end_effector, ros::Time::now(), ros::Duration(1.0));
        tf::StampedTransform transform;
        double x,y,z,rx,ry,rz,rw;
        try {
            // Get the transform from "base_link" to "wrist_3_link"
            tf_.lookupTransform(base_link, end_effector, ros::Time(0), transform);
            response.posx.push_back(transform.getOrigin().getX());
            response.posx.push_back(transform.getOrigin().getY());
            response.posx.push_back(transform.getOrigin().getZ());
            response.posx.push_back(transform.getRotation().getX()); 
            response.posx.push_back(transform.getRotation().getY()); 
            response.posx.push_back(transform.getRotation().getZ());
            response.posx.push_back(transform.getRotation().getW());
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Error getting transform: %s", ex.what());
            return false;
        }
        return true;
    }

    bool moveJointSpace(assignment_ur::JointSpaceMotion::Request &request,
                        assignment_ur::JointSpaceMotion::Response &response)
    {
        // Step1: get current state 
        auto joint_state = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/eff_joint_traj_controller/state", nh, ros::Duration(1.0));
        auto start = std::move(request.joints1);
        auto end   = std::move(request.joints2);
        auto max_vel = request.joints_vel;
        auto max_acc = request.joints_acc;
        double time_from_start = 3.0; // time to move robot from current position to start position

        // Take current state as start if no start input
        if (start.size() == 0)  
        {
            ROS_INFO("Use current state as start point");
            start = std::move(joint_state->actual.positions);
            time_from_start = 0.0;
        }

        // Check valid condition
        if ((start.size() > 0 && start.size() != joint_state->joint_names.size()) || end.size() != joint_state->joint_names.size()) {
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
        traj_generator->setParameters(start, end, max_vel, max_acc, time_step);

        // make a goal trajecotry 
        trajectory_msgs::JointTrajectory goal_traj;
        goal_traj.joint_names = std::move(joint_state->joint_names);

        // set joint trajectory 
        traj_generator->generateJointSpaceTrajectory(goal_traj, time_from_start);

        // Publish trajectory
        goal_joint_pub.publish(goal_traj);
        
        response.result = true;
        return true;
    }

    bool moveCartesianSpace(assignment_ur::CartesianSpaceMotion::Request &request,
                        assignment_ur::CartesianSpaceMotion::Response &response)
    {
        auto joint_state = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/eff_joint_traj_controller/state", nh, ros::Duration(1.0));
        auto start = std::move(request.point1);
        auto end   = std::move(request.point2);
        auto max_vel = request.linear_vel;
        auto max_acc = request.linear_acc;
        double time_from_start = 3.0; // time to move robot from current position to start position

        // Take current position as start if no start input
        if (start.size() == 0)  
        {
            ROS_INFO("Use current state as start point");

            // get current cart_pos 
            tf::TransformListener tf_;
            tf_.waitForTransform(base_link, end_effector, ros::Time::now(), ros::Duration(0.5));
            tf::StampedTransform transform;
            double x,y,z;
            try {
                // Get the transform from "base_link" to "wrist_3_link"
                tf_.lookupTransform(base_link, end_effector, ros::Time(0), transform);
                x = transform.getOrigin().getX();
                y = transform.getOrigin().getY();
                z = transform.getOrigin().getZ();
                
                // Print the transform
                ROS_INFO("Translation: [%.3f, %.3f, %.3f]", x,y,z);
                ROS_INFO("Rotation: [%.3f, %.3f, %.3f, %.3f]", transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ(), transform.getRotation().getW());
            } catch (tf::TransformException& ex) {
                ROS_ERROR("Error getting transform: %s", ex.what());
                response.result = false;
                return false;
            }

            start.push_back(x);
            start.push_back(y);
            start.push_back(z);
            time_from_start = 0.0;
        }

        // Check valid condition
        if ((start.size() > 0 && start.size() != num_linear) || end.size() != num_linear) {
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
        traj_generator->setParameters(start, end, max_vel, max_acc, time_step);

        // make a goal trajecotry 
        trajectory_msgs::JointTrajectory goal_traj;
        goal_traj.joint_names = std::move(joint_state->joint_names);

        std::vector<double> current_jnt_pos(joint_state->actual.positions);
        traj_generator->generateCartesianSpaceTrajectory(*ik_solver, goal_traj, current_jnt_pos, time_from_start);

        // Publish trajectory
        goal_joint_pub.publish(goal_traj);
        
        response.result = true;
        return true;
    }


private:
    ros::NodeHandle nh; // rosnode handle

    // Service of UR5 
    ros::ServiceServer move_home_sv;
    ros::ServiceServer get_state_sv; // service get robot state
    ros::ServiceServer move_joint_space_sv; 
    ros::ServiceServer move_cartesian_space_sv;

    // Publisher and action for control commands
    ros::Publisher goal_joint_pub;
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> move_home_client;

    // Trajectory generator and IK Solver
    std::shared_ptr<trajectory_generator::TrajectoryGenerator> traj_generator; 
    std::shared_ptr<trajectory_generator::IKSolver> ik_solver;
    
    // Other parameters
    double time_step = 0.2;
    std::string base_link = "base_link";
    std::string end_effector = "tool0";
    int num_joint = 6; // number of joints
    int num_linear = 3; // assume only xyz cartestian input

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur5_server");
    UR5_Service ur5_server;
    ros::spin();
}