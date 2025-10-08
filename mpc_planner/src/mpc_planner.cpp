#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <cmath>
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include <cstring>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mpc_planner/mpc_planner.h>
#include <casadi/casadi.hpp>



PLUGINLIB_EXPORT_CLASS(mpc_planner::MpcPlanner, nav_core::BaseLocalPlanner)


namespace mpc_planner {

    namespace cs = casadi;

    MpcPlanner::MpcPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false), listener(tfBuffer){}

    MpcPlanner::MpcPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false), listener(tfBuffer)
    {
        initialize(name, tf, costmap_ros);
    }

    MpcPlanner::~MpcPlanner() {}

    void MpcPlanner::buildSolver(){
        cs::MX X = cs::MX::sym("X", nx*(Np+1));                 // States: [x0,y0,th0,...,xN,yN,thN]
        cs::MX U = cs::MX::sym("U", nu*Np);                     // Inputs: [v0, w0, v1, w1, ...]
        cs::MX p = cs::MX::sym("p", nx + 3*(Np + 1) + 8);       // parameters of the optimization problem: [x0,y0,th0, reference(3*(N+1)), Qx, Qy, Qth, Rv, Rw, Px, Py, Pth]
        

        // Helper: indice del blocco pesi dentro p
        int weights_start_idx = nx + nx*(Np + 1);

        // Extraction of the state from p
        cs::MX x = cs::MX::zeros(3);
        x(0) = p(0); 
        x(1) = p(1); 
        x(2) = p(2);

        // Extraction of the weights from p
        cs::MX Qx = p(weights_start_idx + 0);
        cs::MX Qy = p(weights_start_idx + 1);
        cs::MX Qth = p(weights_start_idx + 2);
        cs::MX Rv = p(weights_start_idx + 3);
        cs::MX Rw = p(weights_start_idx + 4);
        cs::MX Px = p(weights_start_idx + 5);
        cs::MX Py = p(weights_start_idx + 6);
        cs::MX Pth = p(weights_start_idx + 7);

        // Constraint function
        std::vector<cs::MX> g;

        // Stato iniziale vincolato al robot
        cs::MX x0 = X(cs::Slice(0, nx));
        cs::MX x_init = p(cs::Slice(0, nx));   // primo nx valori di p
        g.push_back(x0 - x_init);

        // Vincoli dinamici
        for (int k = 0; k < Np; ++k) {
            cs::MX xk = X(cs::Slice(nx*k, nx*(k+1)));
            cs::MX xk1 = X(cs::Slice(nx*(k+1), nx*(k+2)));
            cs::MX vk = U(2*k + 0);
            cs::MX wk = U(2*k + 1);

            cs::MX dyn = cs::MX::vertcat(std::vector<cs::MX>{
                xk1(0) - (xk(0) + dt*vk*cs::MX::cos(cs::MX(xk(2)))),
                xk1(1) - (xk(1) + dt*vk*cs::MX::sin(cs::MX(xk(2)))),
                xk1(2) - (xk(2) + dt*wk)
            });

            g.push_back(dyn);
        }


        // Cost function
        cs::MX J = cs::MX::zeros(1);
        auto ref_idx = [&](int k,int i){ return 3 + 3*k + i; };

        for (int k = 0; k < Np; ++k) {
            cs::MX xk = X(cs::Slice(nx*k, nx*(k+1)));
            cs::MX x_r = cs::MX::vertcat(std::vector<cs::MX>{
                p(ref_idx(k,0)), 
                p(ref_idx(k,1)), 
                p(ref_idx(k,2))
            });
            cs::MX vk = U(2*k + 0);
            cs::MX wk = U(2*k + 1);

            J = J + Qx*(xk(0)-x_r(0))*(xk(0)-x_r(0))
                + Qy*(xk(1)-x_r(1))*(xk(1)-x_r(1))
                + Qth*(xk(2)-x_r(2))*(xk(2)-x_r(2))
                + Rv*vk*vk + Rw*wk*wk;
        }

        // Terminal cost
        cs::MX xN = X(cs::Slice(nx*Np, nx*(Np+1)));
        cs::MX x_rN = cs::MX::vertcat(std::vector<cs::MX>{
            p(ref_idx(Np,0)), 
            p(ref_idx(Np,1)), 
            p(ref_idx(Np,2))
        });
        J = J + Px*(xN(0)-x_rN(0))*(xN(0)-x_rN(0))
            + Py*(xN(1)-x_rN(1))*(xN(1)-x_rN(1))
            + Pth*(xN(2)-x_rN(2))*(xN(2)-x_rN(2));

        // **** CONSTRAINTS *****
        int nX = nx*(Np+1);
        int nU = nu*Np;
        int n_opt = nX + nU;           // dimensione di opt_vars

        lbx_full = cs::DM::ones(n_opt) * -1e20;
        ubx_full = cs::DM::ones(n_opt) *  1e20;

        // State constraints:
        for (int i = 0; i < nX; ++i) {
            lbx_full(i) = -1e20;
            ubx_full(i) =  1e20;
        }

        // Input constraints)
        for (int k = 0; k < Np; ++k) {
            lbx_full(nX + 2*k + 0) = v_min;
            ubx_full(nX + 2*k + 0) = v_max;
            lbx_full(nX + 2*k + 1) = w_min;
            ubx_full(nX + 2*k + 1) = w_max;
        }


        int ng = nx*(Np+1);
        lbg = cs::DM::zeros(ng);
        ubg = cs::DM::zeros(ng);

        // Variabili decisionali totali = X + U
        cs::MX opt_vars = cs::MX::vertcat(std::vector<cs::MX>{X,U});

        // NLP
        std::map<std::string, cs::MX> nlp = { {"x", opt_vars}, {"f", J}, {"g", cs::MX::vertcat(g)}, {"p", p} };

        // Solver setting
        cs::Dict opts;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        opts["ipopt.tol"] = 1e-3;
        opts["ipopt.max_iter"] = 50;

        solver_ = nlpsol("solver", "ipopt", nlp, opts);

        U_previous = cs::DM::zeros(nu*Np);
        X_previous = cs::DM::zeros(nx*(Np+1));

        // ROS_INFO("DEBUG sizes: nX=%d, nU=%d, n_opt=%d, ng=%d", nX, nU, n_opt, ng);

    }

    void MpcPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
        if (!initialized_){
            nh_ = ros::NodeHandle(name);
            tf_ = tf;

            sub_odom = nh_.subscribe<nav_msgs::Odometry>("/locobot/odom", 1, &MpcPlanner::odomCallback, this);
            pub_cmd = nh_.advertise<geometry_msgs::Twist>("/locobot/cmd_vel", 1);
            pub_optimal_traj = nh_.advertise<nav_msgs::Path>("/locobot/move_base/TrajectoryPlannerROS/local_plan", 1);
            
            buildSolver();

            // dynamic reconfig
            // dynamic_reconfig_.setCallback([this](const DynamicReconfig::Config& cfg){
            //     // apply runtime changes to mpc_core_ or cost function
            //     mpc_core_->setWeights(cfg.q_weights, cfg.r_weights);
            //     if (cfg.horizon > 0) {
            //         horizon_ = cfg.horizon;
            //         mpc_core_->setHorizon(horizon_, cfg.dt);
            //     }
            // });

            initialized_ = true;
            ROS_INFO("MPC local planner initialized");
        }
        else{
            return;
        }
    }


    void MpcPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        /*
            Funzione di callback che viene eseguita quando il plugin si sottoscrive e riceve il messaggio di odometria.
        */

        current_odom_ = *msg;
        // ROS_INFO("Odometry riceived!");
    }


    bool MpcPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {

        /*
            Funzione già presente di default. Viene eseguita:
                - la prima volta quando viene dato il goal;
                - successivamente con la "planner frequency" specificata nel file move_base_params.yaml (interbotix_xslocobot_nav/config)
        */
        
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        global_plan_.clear();
        global_plan_ = orig_global_plan;

        int global_plan_size = global_plan_.size();

        std::cout << "Global Plan size: " << global_plan_size << std::endl;
        
        
        //quando settiamo un nuovo goal (planner frequency 0 Hz nel config file .yaml -> global planner chiamato una volta, solo all'inizio), 
        //resettiamo il flag. In questo modo in seguito potremo eseguire delle verifiche per capire se il goal è stato raggiunto
        goal_reached_=false;

        //Salviamo quindi solo l'ultimo punto del vettore che contiene tutto il global path
        int size_global_plan=global_plan_.size();
        goal_pose_=global_plan_[size_global_plan-1];

        // Save final goal position and orientation
        goal_pos << goal_pose_.pose.position.x,  goal_pose_.pose.position.y;
        goal_orient = tf2::getYaw(goal_pose_.pose.orientation);

        
        std::cout << "COORDINATE GOAL RICEVUTE: " << std::endl;
        std::cout << "Pose Frame : " << goal_pose_.header.frame_id << std::endl; //FRAME = "map" (coincide con /odom)
        std::cout << "  Coordinates (meters) : " << goal_pos[0] << " " << goal_pos[1] << std::endl;
        std::cout << "  Orientation z-axis (radians) : " << goal_orient << std::endl;


        return true;
    }

    bool MpcPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        if(global_plan_.empty()){
            ROS_ERROR("Global Plan is empty");
            return false;
        }

        // Extract odometry information from the "current_odom_" message
        double x = current_odom_.pose.pose.position.x;
        double y = current_odom_.pose.pose.position.y;
        double theta = tf2::getYaw(current_odom_.pose.pose.orientation);

        double v = current_odom_.twist.twist.linear.x;
        double w = current_odom_.twist.twist.angular.z;


        // Convert pose to state vector (x, y, theta)
        Eigen::Vector3d x_state(x, y, theta);
        Eigen::Vector2d current_rob_pos(x,y);
        double current_rob_orient = theta;

        Eigen::Vector2d dist_from_goal = current_rob_pos - goal_pos;
        
        if(dist_from_goal.norm() <= distance_tolerance){
            std::cout << "------------- Distanza raggiunta ----------------" << std::endl;

            Eigen::Vector2d u_opt(0,0);
            goal_reached_=true;


        }
        else{
            std::cout << "------------- Distanza non raggiunta ----------------" << std::endl;

            
            cs::DM p = cs::DM::zeros(nx + nx*(Np + 1) + 8);
            p(0) = x;
            p(1) = y; 
            p(2) = theta;

            // Extract the closesest point of the global plan


            // Build a simple reference trajectory from the first few waypoints
            // std::vector<Eigen::Vector2d> ref_traj;

            buildReferenceTrajectory(p, Np, x, y);
            
            // for (int k = 0; k < std::min(Np, (int)global_plan_.size()); ++k) {
            //     double x_waypoint = global_plan_[k].pose.position.x;
            //     double y_waypoint = global_plan_[k].pose.position.y;
                
            //     // Eigen::Vector2d waypoint(x_waypoint, y_waypoint);
            //     // ref_traj.push_back(waypoint);

            //     p(3 + 3*k + 0) = x_waypoint;
            //     p(3 + 3*k + 1) = y_waypoint;
            //     p(3 + 3*k + 2) = 0;
                
            // }

            

            double Qx = 20;
            double Qy = 20;
            double Qth = 0;
            double Rv = 2;
            double Rw = 2;
            double Px = 0;
            double Py = 0;
            double Pth = 0;

            int weights_start_idx = nx + nx*(Np + 1);
            
            p(weights_start_idx + 0) = Qx;
            p(weights_start_idx + 1) = Qy;
            p(weights_start_idx + 2) = Qth;
            p(weights_start_idx + 3) = Rv;
            p(weights_start_idx + 4) = Rw;
            p(weights_start_idx + 5) = Px;
            p(weights_start_idx + 6) = Py;
            p(weights_start_idx + 7) = Pth;


            // prepara arg e chiama solver (warm-start se vuoi)
            cs::DM x0 = cs::DM::vertcat(std::vector<cs::DM>{X_previous, U_previous});
            std::map<std::string, cs::DM> arg;
            arg["x0"] = x0;
            arg["lbx"] = lbx_full;
            arg["ubx"] = ubx_full;
            arg["lbg"] = lbg;
            arg["ubg"] = ubg;
            arg["p"]  = p;

            // ROS_INFO("DEBUG arg sizes: x0=%d, lbx_full=%d, ubx_full=%d, lbg=%d, ubg=%d, p=%d", 
            //     (int)x0.numel(), 
            //     (int)lbx_full.numel(), 
            //     (int)ubx_full.numel(), 
            //     (int)lbg.numel(), 
            //     (int)ubg.numel(), 
            //     (int)p.numel());

            std::map<std::string, cs::DM> res = solver_(arg);
            cs::DM solution = res.at("x");

            cs::DM X_opt = solution(casadi::Slice(0, nx*(Np+1)));                // primi nx*(Np+1) valori → stati
            cs::DM U_opt = solution(casadi::Slice(nx*(Np+1), nx*(Np+1)+nu*Np));  // restanti nu*Np valori → controlli

            // Solve MPC optimization problem
            //Eigen::Vector2d u_opt(0,0);     //solveOptimizationProblem(x0, ref_traj);

          
            // Extracion of the first control input
            cmd_vel.linear.x = double(U_opt(0));
            cmd_vel.angular.z = double(U_opt(1));
        
            // cmd_vel.linear.x = 0.0;
            // cmd_vel.angular.z = 0.0;
            

            pub_cmd.publish(cmd_vel);

            X_previous = X_opt;
            U_previous = U_opt;


            nav_msgs::Path path_msg;
            path_msg.header.stamp = ros::Time::now();
            // usa il frame della global plan/goal se lo hai, altrimenti "odom"
            std::string frame = "odom";
            if (!goal_pose_.header.frame_id.empty()) frame = goal_pose_.header.frame_id;
            path_msg.header.frame_id = frame;

            for (int k = 0; k <= Np; ++k) {
                // indice base nello slice X_opt (X_opt è vettore [x0,y0,th0, x1,y1,th1, ...])
                int base = nx * k;
                double xk = double(X_opt(base + 0));
                double yk = double(X_opt(base + 1));
                double thk = double(X_opt(base + 2));

                geometry_msgs::PoseStamped ps;
                ps.header = path_msg.header;
                ps.header.stamp = ros::Time::now(); // o la stessa stamp di path_msg
                ps.pose.position.x = xk;
                ps.pose.position.y = yk;
                ps.pose.position.z = 0.0;

                tf2::Quaternion q;
                q.setRPY(0.0, 0.0, thk);
                ps.pose.orientation = tf2::toMsg(q);

                path_msg.poses.push_back(ps);
            }

            pub_optimal_traj.publish(path_msg);

            std::cout << "\nmessaggio pubblicato\n" << std::endl;
            std::cout << "Linear velocity: " << cmd_vel.linear.x  << std::endl;
            std::cout << "Angular velocity: " << cmd_vel.angular.z  << std::endl;
            std::cout << "\n-------------------------------------------------------------------\n\n\n" << std::endl;

        }

        return true;
    }

    bool MpcPlanner::isGoalReached() {
        return goal_reached_;
    }

    void MpcPlanner::buildReferenceTrajectory(cs::DM& p_, int Np_, double cur_x, double cur_y) {
        // 1️Trova il punto della traiettoria più vicino al robot
        int closest_idx = 0;
        double min_dist2 = std::numeric_limits<double>::max();

        for (size_t i = 0; i < global_plan_.size(); ++i) {
            double dx = global_plan_[i].pose.position.x - cur_x;
            double dy = global_plan_[i].pose.position.y - cur_y;
            double dist2 = dx*dx + dy*dy;
            if (dist2 < min_dist2) {
                min_dist2 = dist2;
                closest_idx = i;
            }
        }

        // 2️Copia Np punti a partire dal closest_idx
        for (int k = 0; k < Np_; ++k) {
            int traj_idx = closest_idx + k;

            if (traj_idx < global_plan_.size()) {
                // copia il punto dalla traiettoria
                p_(3 + 3*k + 0) = global_plan_[traj_idx].pose.position.x;
                p_(3 + 3*k + 1) = global_plan_[traj_idx].pose.position.y;
                p_(3 + 3*k + 2) = 0.0; // theta, puoi calcolarlo se vuoi
            } else {
                // siamo alla fine: ripeti l'ultimo punto (goal)
                int last = global_plan_.size() - 1;
                p_(3 + 3*k + 0) = global_plan_[last].pose.position.x;
                p_(3 + 3*k + 1) = global_plan_[last].pose.position.y;
                p_(3 + 3*k + 2) = 0.0;
            }
        }
    }

    // void MpcPlanner::loadParameters() {
    //     nh_.param("~control_dt", control_dt_, 0.1);
    //     nh_.param("~horizon", horizon_, 10);
    // }

} // namespace mpc_planner

