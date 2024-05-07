#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace tracjectory_generator
{

struct Trajectory {
    std::vector<std::vector<double>> total_points;
    std::vector<std::vector<double>> velocities;
    std::vector<std::vector<double>> accelerations;
    std::vector<double> time_stamps;
};

struct SingleTrajectory {
    std::vector<double> points;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> time_stamps;
};

class IKSolver
{
public:
    IKSolver(std::string urdf_file, int num_joint, std::string frame1, std::string frame2);
    ~IKSolver();

    int findIKJointGoalPoint(const std::vector<double>& current_jnt_pos, 
                              const std::vector<double>& target_cart_pos,
                              const std::vector<double>& target_cart_vel,
                              std::vector<double>& target_jnt_pos,
                              std::vector<double>& target_jnt_vel);

private:
    // URDF tree
    std::shared_ptr<KDL::Tree> ur5_tree_;
    std::shared_ptr<KDL::Chain> ur5_chain_;
    int num_jnt;
    std::string base_link;
    std::string end_ee;

    // Solver
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_; 
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> vel_ik_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_NR> pos_ik_solver_;

}; // Invert Kinematic solver class


class TrajectoryGenerator
{
public:
    TrajectoryGenerator();
    ~TrajectoryGenerator();
    
    Trajectory getTrajectory();
    void clearTrajectory();
    void setParameters(const std::vector<double>& start, const std::vector<double>& end,
                        double max_vel, double max_acc, double timestep);
    void generateTrapezoidalTrajectory();
    void generateJointSpaceTrajectory(trajectory_msgs::JointTrajectory& traj_out, double time_from_start = 0.0);
    void generateCartesianSpaceTrajectory(IKSolver& ik_solver, 
                                          const std::vector<double>& current_joint_pos,
                                          trajectory_msgs::JointTrajectory& traj_out, 
                                          double time_from_start = 0.0);

private:
    std::shared_ptr<std::vector<std::shared_ptr<KDL::VelocityProfile_Trap>>> trap_generators_;
    std::shared_ptr<Trajectory> final_trajectory_;
    std::vector<double> start_point, end_point;
    double max_velocity, max_acceleration;
    double time_step;

    void transposeMatrix(std::vector<std::vector<double>>& matrix);

}; // trajectory generation class

} // namespace trajectory_generator

#endif