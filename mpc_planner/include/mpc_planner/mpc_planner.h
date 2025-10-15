#pragma once
#include <memory>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <Eigen/Dense>
#include <base_local_planner/odometry_helper_ros.h>
#include <casadi/casadi.hpp>

#include <mpc_planner/classes.h>



namespace mpc_planner {


class MpcPlanner : public nav_core::BaseLocalPlanner {

public:
    MpcPlanner();
    MpcPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    ~MpcPlanner();
    

    // Standard methods of base_local_planner plugin
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
    
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    
    bool isGoalReached();

    // MPC methods
    //void setWeights(const Eigen::VectorXd& Q_, const Eigen::VectorXd& R_, const Eigen::VectorXd& P_);               // set cost weights (q: state weights flattened; r: control weights flattened)
    void buildSolver();
    void buildReferenceTrajectory(casadi::DM& p, int Np, double cur_x, double cur_y);
    void loadParameters();

    // Odometry 
    void extractOdometryInfo();

    // Object info extraction
    
    


private:
    // ROS information
    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;
    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_;
    
    nav_msgs::Odometry current_odom_;
    
    ros::Subscriber sub_odom;
    ros::Subscriber sub_obs;
    ros::Publisher pub_cmd;
    ros::Publisher pub_optimal_traj;
    ros::Publisher pub_ref_posearray;


    // Global plan information
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    geometry_msgs::PoseStamped goal_pose_;
    bool initialized_ = false;
    bool goal_reached_ = false;


    // MPC parameters
    int nx = 3;             // state dim
    int nu = 2;             // control dim
    int ns = 1;             // slack variable dimension
    int Np = 40;            // prediction horizon
    int Nc = 0;             // control horizon
    int N_cost_params = 0;
    int N_obs = 0;
    int N_obs_info = 5;
    int ref_len = nx*(Np+1);
    std::string model = "euler";
    double r_robot = 0.5;
    double dt = 0.05;        // Timestep
    double v_max = 0.5;
    double v_min = 0;
    double w_max = 1.5;
    double w_min = -1.5;
    double delta_v_max = 0.1;  // [m/s per step] esempio: variazione massima velocità lineare
    double delta_w_max = 0.5;  // [rad/s per step] esempio: variazione massima velocità angolare

    casadi::Function solver_; // il solver CasADi (nlpsol)
    casadi::DM lbx_full, ubx_full;    // bounds su decision vars
    casadi::DM lbg, ubg;    // bounds su decision vars
    casadi::DM U_previous;       // warm-start (soluzione precedente)
    casadi::DM X_previous;
    casadi::DM s_previous;

    Eigen::Vector3d Q;  // state weights
    Eigen::Vector2d R;  // control weights
    Eigen::Vector3d P;  // final state weights


    // Planner infomation
    Eigen::Vector2d goal_pos;
    double goal_orient;
    double distance_tolerance=0.2;
    double angle_tolerance=0.13;
    

    // Obstacle information
    std::vector<Obstacle> obstacles_list;
    double max_n_obs = 2;

    // Callback functions
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void obstacleCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

    };


} // namespace mpc_planner

