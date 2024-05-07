#include "trajectory_generator.hpp"

namespace tracjectory_generator
{
TrajectoryGenerator::TrajectoryGenerator():
    start_point({}),
    end_point({}),
    max_velocity(0.0),
    max_acceleration(0.0),
    time_step(0.0),
    final_trajectory_(std::make_shared<Trajectory>()),
    trap_generators_(std::make_shared<KDL::VelocityProfile_Trap>()) {}

Trajectory TrajectoryGenerator::getTrajectory()
{
    return *final_trajectory_;
}

void TrajectoryGenerator::setParameters(const std::vector<double>& start, const std::vector<double>& end,
                                        double max_vel, double max_acc, double timestep)
{
    start_point = start;
    end_point = end;
    max_velocity = max_vel;
    max_acceleration = max_acc;
    time_step = timestep;
    trap_generators_->SetMax(max_vel, max_acc);
}

void TrajectoryGenerator::clearTrajectory()
{
    final_trajectory_->total_points.clear();
    final_trajectory_->velocities.clear();
    final_trajectory_->accelerations.clear();
    final_trajectory_->time_stamps.clear();
}

void TrajectoryGenerator::generateTrapezoidalTrajectory()
{
    if (max_velocity==0 || max_acceleration==0 || time_step==0 || start_point.size()==0 || end_point.size()==0) {
        std::cerr << "Please check the setup" << std::endl;
        return;
    }

    clearTrajectory(); // clear history trajectory
    size_t num_coor = start_point.size(); // number of coordinates
    
    // Find the maximum duration of trajectory in each coordinate
    double max_duration = 0;
    for (size_t i = 0; i < num_coor; ++i) {
        double pos1 = start_point[i];
        double pos2 = end_point[i];
        trap_generators_->SetProfile(pos1, pos2);
        double duration = trap_generators_->Duration();  
        if (duration > max_duration) max_duration = duration;      
    }

    // Generate trapezoidal trajectory according to max_duration
    for (size_t i = 0; i < num_coor; ++i) {
        SingleTrajectory single_trajectory;

        // Generate profile
        double pos1 = start_point[i];
        double pos2 = end_point[i];
        trap_generators_->SetProfileDuration(pos1, pos2, max_duration);

        // Get point of each timestep in trajectory
        for (double time = 0; time < max_duration; time += time_step) {
            single_trajectory.points.push_back(trap_generators_->Pos(time));
            single_trajectory.velocity.push_back(trap_generators_->Vel(time));
            single_trajectory.acceleration.push_back(trap_generators_->Acc(time));
            single_trajectory.time_stamps.push_back(time);
        }

        // Add the end point
        single_trajectory.points.push_back(trap_generators_->Pos(max_duration));
        single_trajectory.velocity.push_back(trap_generators_->Vel(max_duration));
        single_trajectory.acceleration.push_back(trap_generators_->Acc(max_duration));
        single_trajectory.time_stamps.push_back(max_duration);

        // Add coordinate traj to final_trajectory
        final_trajectory_->total_points.push_back(single_trajectory.points);
        final_trajectory_->velocities.push_back(single_trajectory.velocity);
        final_trajectory_->accelerations.push_back(single_trajectory.acceleration);
        final_trajectory_->time_stamps = single_trajectory.time_stamps;
    }

    // transpose the final_trajectory 
    transposeMatrix(final_trajectory_->total_points);
    transposeMatrix(final_trajectory_->velocities);
    transposeMatrix(final_trajectory_->accelerations);
}

void TrajectoryGenerator::generateJointSpaceTrajectory(trajectory_msgs::JointTrajectory& traj_out)
{
    if (max_velocity==0 || max_acceleration==0 || time_step==0) {
        std::cerr << "Please check the setup" << std::endl;
        return;
    }

    // Generate joint trajectory 
    generateTrapezoidalTrajectory();

    // Convert final_traj to trajectory_msgs
    std::vector<double> time_stamps = final_trajectory_->time_stamps; // number of time steps
    size_t num_joints = final_trajectory_->total_points.size(); // number of joints

    // Create JointTrajectoryPoint in each timestep and add to output trajectory
    for (const double& time: time_stamps)
    {
        trajectory_msgs::JointTrajectoryPoint joint_traj_point;
        joint_traj_point.positions = final_trajectory_->total_points[time];
        joint_traj_point.velocities = final_trajectory_->velocities[time];
        joint_traj_point.time_from_start = ros::Duration(time);

        traj_out.points.push_back(joint_traj_point);
    } 
}


void TrajectoryGenerator::transposeMatrix(std::vector<std::vector<double>>& matrix)
{
    if (matrix.size() == 0) {
        std::cerr << "Invalid matrix" << std::endl;
        return;
    }

    // Get the number of rows and columns of the original matrix
    size_t rows = matrix.size();
    size_t cols = matrix[0].size();

    // Perform the transpose
    std::vector<std::vector<double>> temp(cols, std::vector<double>(rows));
    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            temp[j][i] = matrix[i][j];
        }
    }

    // copy the transposed matrix back to the original matrix
    matrix = std::move(temp);
}

} // Trajectory_generator
