#include <iostream>
#include "trajectory_generator.hpp"

int main()
{
    std::vector<std::vector<double>> a{{1,2},{1,2},{1,2}};

    tracjectory_generator::TrajectoryGenerator traj_gen;
    std::vector<double> start{1,1};
    std::vector<double> end{3,2};
    double max_vel = 3;
    double max_acc = 4;
    double time_step = 0.1;
    
    // setup
    traj_gen.setParameters(start,end,max_vel,max_acc,time_step);
    // generate trajectory 
    traj_gen.generateTrapezoidalTrajectory();

    // 
    tracjectory_generator::Trajectory trajectory = traj_gen.getTrajectory();

    for (auto& step : trajectory.velocities) {
        for (auto& pos : step) {
            std::cout << pos << " ";
        }
        std::cout << std::endl;
    }

    return 0;
    
}