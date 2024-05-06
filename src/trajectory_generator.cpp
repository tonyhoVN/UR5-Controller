#include "trajectory_generator.hpp"

namespace tracjectory_generator
{
TrajectoryGenerator::TrajectoryGenerator(const std::vector<double>& start, const std::vector<double>& end,
    double max_vel, double max_acc, double timestep):
    start_point({}),
    end_point({}),
    max_velocity(0.0),
    max_acceleration(0.0),
    time_step(0.0),
    final_trajectory_(std::make_shared<Trajectory>()),
    trap_generators_(std::make_shared<KDL::VelocityProfile_Trap>()) 
{
    trap_generators_->SetMax(max_vel, max_acc);
}

Trajectory TrajectoryGenerator::getTrajectory()
{
    return *final_trajectory_;
}

void TrajectoryGenerator::setParameters()
{
    
}

void TrajectoryGenerator::clearTrajectory()
{
    final_trajectory_->total_points.clear();
    final_trajectory_->velocities.clear();
    final_trajectory_->accelerations.clear();
    final_trajectory_->time_stamps.clear();
    start_point.clear();
    end_point.clear();
}

void TrajectoryGenerator::generateTrapezoidalTrajectory()
{
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
}


} // Trajectory_generator
