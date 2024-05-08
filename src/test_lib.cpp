#include <iostream>
#include "trajectory_generator.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include "sensor_msgs/JointState.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_ur5");
    ros::NodeHandle nh;
    ros::Publisher goal_joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 1);
    trajectory_generator::TrajectoryGenerator traj_gen;

    std::string base_link = "base_link";
    std::string end_effector = "tool0";

    ///////////////////////////////////////////////////////////////////

    // // get current state
    // auto joint_state = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh, ros::Duration(1.0));

    // // make a goal trajecotry 
    // trajectory_msgs::JointTrajectory goal_traj;

    // // set joint name 
    // goal_traj.joint_names = std::move(joint_state->name);
    // // for (auto name : joint_state->name) {
    // //     goal_traj.joint_names.push_back(name);
    // // }

    // std::vector<double> start;
    // start = joint_state->position;
    // std::vector<double> end{0,0,0,0,0,0};
    // double max_vel = 0.1;
    // double max_acc = 0.2;
    // double time_step = 0.2;
    
    // // setup generator
    // traj_gen.setParameters(start,end,max_vel,max_acc,time_step);

    // traj_gen.generateJointSpaceTrajectory(goal_traj);

    // // for (auto& point : goal_traj.points) {
    // //     for (auto& pos : point.positions) {
    // //         std::cout << pos << " ";
    // //     }
    // //     std::cout << std::endl;
    // // }

    // // publish 
    // goal_joint_pub.publish(goal_traj);


    ///////////////////////////////////////////////////////////////////

    // get ur path
    std::string urdf_path = ros::package::getPath("assignment_ur");
	if(urdf_path.empty()) {
		ROS_ERROR("assignment_ur package path was not found");
        return 1;
	}
	urdf_path += "/urdf/ur5_robot.urdf";

    // create IKSolver 
    trajectory_generator::IKSolver ik_solver(urdf_path, 6, base_link, end_effector);
    
    // get current joint state 
    auto joint_state = ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("/eff_joint_traj_controller/state", nh, ros::Duration(1.0));
    std::vector<double> current_jnt_pos = std::move(joint_state->actual.positions);
    
    // get current cart_pos 
    tf::TransformListener tf_;
    tf_.waitForTransform(base_link, end_effector, ros::Time::now(), ros::Duration(1.0));
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
    }

    // Set target for trajectory planning 
    std::vector<double> start{x, y, z};
    std::vector<double> end{x, y ,z-0.2};
    double max_vel = 0.05;
    double max_acc = 0.05;
    double time_step = 0.2;
    std::vector<double> target_vel{0,0,0};
    std::vector<double> target_pos{0.487502, 0.109205, 0.509818};
    
    // setup generator
    trajectory_msgs::JointTrajectory goal_traj;
    goal_traj.joint_names = std::move(joint_state->joint_names);

    traj_gen.setParameters(start,end,max_vel,max_acc,time_step);
    
    // traj_gen.generateCartesianSpaceTrajectory(ik_solver, goal_traj, current_jnt_pos, target_pos, target_vel,1);
    
    traj_gen.generateCartesianSpaceTrajectory(ik_solver, goal_traj, current_jnt_pos);
    
    goal_joint_pub.publish(goal_traj);
    
    return 0;
    
}