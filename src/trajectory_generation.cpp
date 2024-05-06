#include <vector>
#include <cmath>
#include <iostream>

struct JointPoint {
    std::vector<double> positions;
};

struct SpacePoint {
    std::vector<double> positions;
};

struct Trajectory {
    std::vector<std::vector<double>> total_points;
    std::vector<std::vector<double>> velocities;
    std::vector<std::vector<double>> accelerations;
};

struct SingleTrajectory {
    std::vector<double> points;
    std::vector<double> velocity;
    std::vector<double> acceleration;
};

class TrapezoidalMotion {
public:
    TrapezoidalMotion(const JointPoint& start, const JointPoint& end, double max_vel, double max_acc) : 
        start_point(start), 
        end_point(end), 
        max_velocity(max_vel), 
        max_acceleration(max_acc) {}

    Trajectory generateTrajectory(double timestep) {
        Trajectory trajectory;
        if ((max_velocity==0) || (max_acceleration==0)) return trajectory;

        // Generate trajectory for each joint
        size_t num_joints = start_point.positions.size();
        for (size_t i = 0; i < num_joints; ++i) {
            double distance = end_point.positions[i] - start_point.positions[i];
            double sign = distance > 0 ? 1 : -1;
            double t_acc = max_velocity / max_acceleration; // Time to reach max_velocity
            double d_acc = 0.5*max_acceleration*t_acc*t_acc; // Distance covered during acceleration
            double d_cv = std::abs(distance) - (2*d_acc); // Distance covered at constant velocity

            if (d_cv < 0) { // Not enough space to reach full speed
                d_cv = 0;
                max_velocity = std::sqrt(std::abs(distance) * max_acceleration);
                t_acc = max_velocity / max_acceleration;
                d_acc = 0.5*max_acceleration*t_acc*t_acc;
            }

            double d_dec = d_acc; // Distance covered during deceleration
            double t_dec = t_acc; // Time to decelerate
            double t_cv = d_cv / max_velocity; // Time at constant velocity

            SingleTrajectory single_joint_trajectory = createSingleJointTrajectory(
                timestep, t_acc, t_cv, t_dec, d_acc, d_cv, d_dec, sign*max_velocity, max_acceleration, start_point.positions[i], end_point.positions[i]);

            // Make constraint for end point 
            trajectory.total_points.push_back(single_joint_trajectory.points);
            trajectory.velocities.push_back(single_joint_trajectory.velocity);
            trajectory.accelerations.push_back(single_joint_trajectory.acceleration);
        }

        // Make transpose off all matrix
        transposeTrajectory(trajectory.total_points);
        transposeTrajectory(trajectory.velocities);
        transposeTrajectory(trajectory.accelerations);

        return trajectory;
    }


    SingleTrajectory createSingleJointTrajectory(double timestep, double t_acc, double t_cv, double t_dec,
                                                 double d_acc, double d_cv, double d_dec, double max_vel,
                                                 double max_acc, double start_pos, double end_pos) {
        SingleTrajectory single_joint_trajectory;
        // Acceleration phase
        for (double t = 0; t < t_acc; t += timestep) {
            double pos = start_pos + 0.5*max_vel*t*t; // position
            double vel = max_acc*t;                   // velocity 
            double acc = max_acc;                     // acceleration
            single_joint_trajectory.points.push_back(pos);
            single_joint_trajectory.velocity.push_back(vel);
            single_joint_trajectory.acceleration.push_back(acc);
        }
        // Constant velocity phase
        for (double t = 0; t < t_cv; t += timestep) {
            double pos = start_pos + d_acc + max_vel*t;
            double vel = max_vel;
            double acc = 0;
            single_joint_trajectory.points.push_back(pos);
            single_joint_trajectory.velocity.push_back(vel);
            single_joint_trajectory.acceleration.push_back(acc);
        }
        // Deceleration phase
        for (double t = 0; t <= t_dec; t += timestep) {
            double pos = start_pos + d_acc + d_cv + max_vel*t - 0.5*max_acc*t*t;
            double vel = max_vel - max_acc*t;
            double acc = -max_acc;
            single_joint_trajectory.points.push_back(pos);
            single_joint_trajectory.velocity.push_back(vel);
            single_joint_trajectory.acceleration.push_back(acc);
        }
        
        // End point constraint
        single_joint_trajectory.points.back() = end_pos;
        single_joint_trajectory.velocity.back() = 0;
        single_joint_trajectory.acceleration.back() = 0;

        return single_joint_trajectory;
    }

    void transposeTrajectory(std::vector<std::vector<double>>& trajectory) {
        if (trajectory.empty()) return;

        // Find maximum length of single joint trajectory
        size_t maxLength = 0;
        for (const auto& traj : trajectory) {
            if (traj.size() > maxLength) {
                maxLength = traj.size();
            }
        }

        // Extend the trajectory of other joints
        for (auto& single_trajectory: trajectory) {
            while (single_trajectory.size() < maxLength) {
                auto end_element = single_trajectory.back();
                single_trajectory.push_back(end_element);
            }
        }
        
        // Make a transpose of trajectory
        std::vector<std::vector<double>> transposed(maxLength, std::vector<double>(trajectory.size(), 0));
        for (size_t i = 0; i < trajectory.size(); ++i) {
            for (size_t j = 0; j < trajectory[i].size(); ++j) {
                transposed[j][i] = trajectory[i][j];
            }
        }

        trajectory = transposed;
    }

private:
    JointPoint start_point, end_point;
    double max_velocity, max_acceleration;
};


int main() {
    JointPoint start{{1,1}};
    JointPoint end{{2,2}};
    TrapezoidalMotion motion(start, end, 2, 3);

    Trajectory trajectory = motion.generateTrajectory(0.1);

    for (auto& step : trajectory.velocities) {
        for (auto& pos : step) {
            std::cout << pos << " ";
        }
        std::cout << std::endl;
    }

    return 0;
}
