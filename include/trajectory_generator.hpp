#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <iostream>
// #include <kdl_parser/kdl_parser.hpp>
// #include <kdl/chain.hpp>
// #include <kdl/chainiksolver.hpp>
// #include <kdl/chainjnttojacsolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainiksolverpos_nr.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>
// #include <kdl/frames.hpp>
// #include <kdl/jacobian.hpp>
// #include <kdl/jntarray.hpp>
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

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(const std::vector<double>& start, const std::vector<double>& end,
                        double max_vel, double max_acc, double timestep);

    Trajectory getTrajectory();
    void clearTrajectory();
    void setParameters();
    void generateTrapezoidalTrajectory();
    void generateJointSpaceTrajectory(trajectory_msgs::JointTrajectory& traj_out);

private:
    std::shared_ptr<KDL::VelocityProfile_Trap> trap_generators_;
    std::shared_ptr<Trajectory> final_trajectory_;
    std::vector<double> start_point, end_point;
    double max_velocity, max_acceleration;
    double time_step;

}; // trajectory generation class

class IKSolver
{
public:
    IKSolver();
    ~IKSolver();
}; // Invert Kinematic solver class

} // namespace trajectory_generator

#endif