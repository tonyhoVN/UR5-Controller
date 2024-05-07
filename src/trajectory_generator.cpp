#include "trajectory_generator.hpp"

namespace tracjectory_generator
{

///////// IKSOLVER CLASS /////////////

IKSolver::IKSolver(std::string urdf_file, int num_joint, std::string frame1, std::string frame2):
    num_jnt(num_joint),
    base_link(frame1),
    end_ee(frame2),
    ur5_tree_(std::make_shared<KDL::Tree>()),
    ur5_chain_(std::make_shared<KDL::Chain>())
{
    // Set the UR_tree
    std::cout << "URDF file: " << urdf_file << std::endl;

    if (!kdl_parser::treeFromFile(urdf_file, *ur5_tree_)){
		std::cerr << "Failed to construct kdl tree" << std::endl;
   		return;
	}
    ur5_tree_->getChain(frame1, frame2, *ur5_chain_);

    // Solver 
    fk_solver_     = std::make_shared<KDL::ChainFkSolverPos_recursive>(*ur5_chain_);
    vel_ik_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(*ur5_chain_, 0.0001, 500);
    pos_ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR>(*ur5_chain_, *fk_solver_, *vel_ik_solver_, 500);
}

IKSolver::~IKSolver() {}

int IKSolver::findIKJointGoalPoint(const std::vector<double>& current_jnt_pos, 
                                    const std::vector<double>& target_cart_pos,  
                                    const std::vector<double>& target_cart_vel,
                                    std::vector<double>& target_jnt_pos,
                                    std::vector<double>& target_jnt_vel)
{
    if (current_jnt_pos.size() != num_jnt || target_cart_pos.size() != 3 || target_cart_vel.size() != 3) {
        std::cerr << "Check input value" << std::endl;
        return 1;
    } 

    KDL::JntArray q_current(num_jnt);
    KDL::JntArray q_target(num_jnt);
    KDL::JntArray q_dot_target(num_jnt);

    // Set current joint 
    for (int i=0; i < num_jnt; i++) {q_current(i) = current_jnt_pos[i];}

    // Get current cartesian pos by FK
    KDL::Frame cart_pos_current;
    fk_solver_->JntToCart(q_current, cart_pos_current);

    std::cout << "FK result: " << cart_pos_current.p(0) << " " << cart_pos_current.p(1) << " " << cart_pos_current.p(2) << " " << std::endl;

    // Set cartesian goal point
    KDL::Vector p_cart_pos_target(target_cart_pos[0], target_cart_pos[1], target_cart_pos[2]); // xyz of goal 
    KDL::Vector p_cart_vel_target(target_cart_vel[0], target_cart_vel[1], target_cart_vel[2]); // vel_xyz of goal 
    KDL::Vector r_cart_vel_target(0.0, 0.0, 0.0); // Assuming no rotational velocity
    KDL::Frame cart_pos_target(cart_pos_current.M, p_cart_pos_target);
    KDL::Twist cart_vel_target(p_cart_vel_target, r_cart_vel_target);

    // IK pos
    int result_pos = pos_ik_solver_->CartToJnt(q_current, cart_pos_target, q_target);
    if (result_pos < 0) {
        std::cerr << "Cannot solve IK for position" << std::endl;
        return 2;
    } 

    target_jnt_pos.clear();
    for (int i=0; i < num_jnt; i++) {target_jnt_pos.push_back(q_target(i));}

    // IK vel
    int result_vel = vel_ik_solver_->CartToJnt(q_target, cart_vel_target, q_dot_target);
    if (result_vel < 0) {
        std::cerr << "Cannot solve IK for velocity" << std::endl;
        return 2;
    } 

    target_jnt_pos.clear();
    for (int i=0; i < num_jnt; i++) {target_jnt_pos.push_back(q_target(i));}
    
    return 0;
}


////////// TRAJECTORY GENERATOR CLASS /////////////

TrajectoryGenerator::TrajectoryGenerator():
    start_point({}),
    end_point({}),
    max_velocity(0.0),
    max_acceleration(0.0),
    time_step(0.0),
    final_trajectory_(std::make_shared<Trajectory>()),
    trap_generators_(std::make_shared<std::vector<std::shared_ptr<KDL::VelocityProfile_Trap>>>()) {}

TrajectoryGenerator::~TrajectoryGenerator() {}

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

    // Create kdl_generator for each coordinate
    for (size_t i = 0; i < start_point.size(); i++)
    {
        std::shared_ptr<KDL::VelocityProfile_Trap> generators_ = std::make_shared<KDL::VelocityProfile_Trap>();
        generators_->SetMax(max_vel, max_acc);
        trap_generators_->push_back(generators_);
    }
}

void TrajectoryGenerator::clearTrajectory()
{
    final_trajectory_->total_points.clear();
    final_trajectory_->velocities.clear();
    final_trajectory_->accelerations.clear();
    final_trajectory_->time_stamps.clear();
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

void TrajectoryGenerator::generateTrapezoidalTrajectory()
{
    if (max_velocity==0 || max_acceleration==0 || time_step==0 || start_point.size()==0 || end_point.size()==0) {
        std::cerr << "Please check the setup" << std::endl;
        return;
    }

    clearTrajectory(); // clear history trajectory

    std::cout << "START POINT-----" << std::endl;
    for (auto pos: start_point) std::cout << pos << " ";
    std::cout << std::endl;

    std::cout << "END POINT-----" << std::endl;
    for (auto pos: end_point) std::cout << pos << " ";
    std::cout << std::endl;
    
    size_t num_coor = start_point.size(); // number of coordinates
    
    // Find the maximum duration of trajectory in each coordinate
    double max_duration = -1;
    for (size_t i = 0; i < num_coor; ++i) {
        double pos1 = start_point[i];
        double pos2 = end_point[i];
        auto generator_ = trap_generators_->at(i);
        generator_->SetProfile(pos1, pos2);
        double duration = generator_->Duration();  
        if (duration > max_duration) max_duration = duration;      
    }

    // Generate trapezoidal trajectory according to max_duration
    for (size_t i = 0; i < num_coor; ++i) {
        SingleTrajectory single_trajectory;

        // Generate profile
        double pos1 = start_point[i];
        double pos2 = end_point[i];
        auto generator_ = trap_generators_->at(i);
        generator_->SetProfileDuration(pos1, pos2, max_duration);

        // Get point of each timestep in trajectory
        for (double time = 0; time < max_duration; time += time_step) {
            single_trajectory.points.push_back(generator_->Pos(time));
            single_trajectory.velocity.push_back(generator_->Vel(time));
            single_trajectory.acceleration.push_back(generator_->Acc(time));
            single_trajectory.time_stamps.push_back(time);
        }

        // Add the end point
        single_trajectory.points.push_back(generator_->Pos(max_duration));
        single_trajectory.velocity.push_back(generator_->Vel(max_duration));
        single_trajectory.acceleration.push_back(generator_->Acc(max_duration));
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

void TrajectoryGenerator::generateJointSpaceTrajectory(trajectory_msgs::JointTrajectory& traj_out, double time_from_start)
{
    if (max_velocity==0 || max_acceleration==0 || time_step==0 || start_point.size()==0 || end_point.size()==0) {
        std::cerr << "Please check the setup" << std::endl;
        return;
    }

    // Generate joint trajectory 
    generateTrapezoidalTrajectory();

    // Convert final_traj to trajectory_msgs
    std::vector<double> time_stamps = final_trajectory_->time_stamps; // number of time steps
    size_t num_joints = final_trajectory_->total_points.size(); // number of joints

    // Create JointTrajectoryPoint in each timestep and add to output trajectory
    for (size_t i = 0; i < final_trajectory_->time_stamps.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint joint_traj_point;
        joint_traj_point.positions = final_trajectory_->total_points[i];
        joint_traj_point.velocities = final_trajectory_->velocities[i];
        joint_traj_point.time_from_start = ros::Duration(final_trajectory_->time_stamps[i] + time_from_start);

        traj_out.points.push_back(joint_traj_point);
    } 
}

void TrajectoryGenerator::generateCartesianSpaceTrajectory(IKSolver& ik_solver, 
                                                           const std::vector<double>& current_joint_pos,
                                                           trajectory_msgs::JointTrajectory& traj_out, 
                                                           double time_from_start)
{
    if (max_velocity==0 || max_acceleration==0 || time_step==0 || start_point.size()==0 || end_point.size()==0) {
        std::cerr << "Please check the setup" << std::endl;
        return;
    }

    // Generate cartesian XYZ trajectory  
    generateTrapezoidalTrajectory();

    // Convert Cartesian to Joint
    std::vector<double> time_stamps = final_trajectory_->time_stamps; // number of time steps
    size_t num_coor = final_trajectory_->total_points.size(); // number of cartesian coordinates (3)

    for (size_t i = 0; i < final_trajectory_->time_stamps.size(); i++) {
        
        // Solver IK from Cartestian point to Joint Space
        std::vector<double> target_car_pos(final_trajectory_->total_points[i]);
        std::vector<double> target_car_vel(final_trajectory_->velocities[i]);
        std::vector<double> target_jnt_pos;
        std::vector<double> target_jnt_vel;
        int result = ik_solver.findIKJointGoalPoint(current_joint_pos,
                                       target_car_pos,
                                       target_car_vel,
                                       target_jnt_pos,
                                       target_jnt_vel);

        if (result != 0) continue;
        
        // Make a JointTrajectoryPoint
        trajectory_msgs::JointTrajectoryPoint joint_traj_point;
        joint_traj_point.positions  = target_jnt_pos;
        joint_traj_point.velocities = target_jnt_vel;
        joint_traj_point.time_from_start = ros::Duration(final_trajectory_->time_stamps[i] + time_from_start);
        traj_out.points.push_back(joint_traj_point);
    }
    
}



} // Trajectory_generator
