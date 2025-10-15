#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <cmath>
#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include <cstring>
#include <angles/angles.h>
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
#include <yaml-cpp/yaml.h>
#include <gazebo_msgs/ModelStates.h>
#include <mpc_planner/classes.h>




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

        // States: [x0,y0,th0,...,xN,yN,thN]
        cs::MX X = cs::MX::sym("X", nx*(Np+1));
        
        // Inputs: [v0, w0, v1, w1, ...]
        cs::MX U = cs::MX::sym("U", nu*Np);
        
        // Slack variables for terminal constraints
        cs::MX s = cs::MX::sym("s", ns);

        // parameters of the optimization problem: 
        //      [x0,y0,th0, reference(3*(N+1)), Qx, Qy, Qth, Rv, Rw, Px, Py, Pth, last_v, last_w, obs_info([x_obs1, y_obs1, vx_obs1, vy_obs1, r_obs1], [x_obs2, y_obs2, vx_obs2, vy_obs2, r_obs2]), ...]   
        cs::MX p = cs::MX::sym("p", nx + nx*(Np+1) + N_cost_params + nu + N_obs*N_obs_info, 1);        

        // Helper: indice del blocco pesi dentro p
        int weights_start_idx = nx + nx*(Np+1);

        // Extraction of the weights from p
        cs::MX Qx = p(weights_start_idx + 0);
        cs::MX Qy = p(weights_start_idx + 1);
        cs::MX Qth = p(weights_start_idx + 2);
        cs::MX Rv = p(weights_start_idx + 3);
        cs::MX Rw = p(weights_start_idx + 4);
        cs::MX Px = p(weights_start_idx + 5);
        cs::MX Py = p(weights_start_idx + 6);
        cs::MX Pth = p(weights_start_idx + 7);
          

        // Cost function
        cs::MX J = cs::MX::zeros(1);

        for (int k = 0; k < Np; ++k) {
            cs::MX xk = X(cs::Slice(nx*k, nx*(k+1)));
            cs::MX x_r = cs::MX::vertcat(std::vector<cs::MX>{
                p(nx + nx*k + 0),
                p(nx + nx*k + 1),
                p(nx + nx*k + 2)
            });
            cs::MX vk = U(2*k + 0);
            cs::MX wk = U(2*k + 1);

            cs::MX diff_th = x_r(2) - xk(2);


            J = J + Qx*(x_r(0) - xk(0))*(x_r(0) - xk(0))
                + Qy*(x_r(1) - xk(1))*(x_r(1) - xk(1))
                + Qth*diff_th*diff_th
                + Rv*vk*vk + Rw*wk*wk;



            // Obstacle avoidance
            // for(int j=0; j<N_obs; j++){

            //     cs::MX obstacles_pos = p(cs::Slice(weights_start_idx + N_cost_params + nu + 3*j, weights_start_idx + N_cost_params + nu + 3*j +2));

            //     // std::cout << "Obstacle dimension: " << obstacles_pos.size1() << std::endl;
            //     cs::MX obstacles_r = p(weights_start_idx + N_cost_params + nu + 3*j + 2);
            //     cs::MX dist = cs::MX::sqrt(cs::MX::fmax(cs::MX::sumsqr(xk(cs::Slice(0,2)) - obstacles_pos), 1e-12));
            //     cs::MX delta = cs::MX::fmax(dist, 1e-3);  
            //     cs::MX obstacle_penalty = 2/ cs::MX::pow(delta, 2);

            //     J = J + obstacle_penalty;
            // }
        }

        // Terminal cost
        cs::MX xN = X(cs::Slice(nx*Np, nx*(Np+1)));
        cs::MX x_rN = cs::MX::vertcat(std::vector<cs::MX>{
            p(nx + nx*Np + 0),
            p(nx + nx*Np + 1),
            p(nx + nx*Np + 2)
        });
        double slack_penalty = 30;

        cs::MX diff_th_N = x_rN(2) - xN(2);

        // J = J + Px*(x_rN(0) - xN(0))*(x_rN(0) - xN(0))
        //     + Py*(x_rN(1) - xN(1))*(x_rN(1) - xN(1))
        //     + Pth*diff_th_N*diff_th_N
        //     + slack_penalty*s*s;


        
        // CONSTRAINTS DEFINITION
        //  - initial condition
        //  - dynamics
        //  - slack variables
        //  

        std::vector<cs::MX> g;                      // Constraints function

        // Stato iniziale vincolato al robot
        cs::MX x0 = X(cs::Slice(0, nx));
        cs::MX x_init = p(cs::Slice(0, nx));   // primo nx valori di p
        
        g.push_back(x0 - x_init);
        lbg = cs::DM::zeros(nx);
        ubg = cs::DM::zeros(nx);


        for (int k = 0; k < Np; ++k) {

            cs::MX dyn;
            cs::MX xk  = X(cs::Slice(nx*k, nx*(k+1)));
            cs::MX xk1 = X(cs::Slice(nx*(k+1), nx*(k+2)));
            cs::MX vk  = U(2*k + 0);
            cs::MX wk  = U(2*k + 1);


            if (model=="euler"){
                // Dynamic constraints (Euler)
                cs::MX x_next = xk(0) + dt * vk * cs::MX::cos(xk(2));
                cs::MX y_next = xk(1) + dt * vk * cs::MX::sin(xk(2));
                cs::MX th_next = xk(2) + dt * wk;

                dyn = cs::MX::vertcat({xk1(0) - x_next,
                                        xk1(1) - y_next,
                                        xk1(2) - th_next});
            }
            else if(model=="RK4"){
                // Modello RK4
                cs::MX dx1 = vk * cs::MX::cos(xk(2));
                cs::MX dy1 = vk * cs::MX::sin(xk(2));
                cs::MX dth1 = wk;
                cs::MX k1 = cs::MX::vertcat({dx1, dy1, dth1});

                cs::MX x_temp = xk + (dt/2) * k1;
                cs::MX dx2 = vk * cs::MX::cos(x_temp(2));
                cs::MX dy2 = vk * cs::MX::sin(x_temp(2));
                cs::MX dth2 = wk;
                cs::MX k2 = cs::MX::vertcat({dx2, dy2, dth2});

                x_temp = xk + (dt/2) * k2;
                cs::MX dx3 = vk * cs::MX::cos(x_temp(2));
                cs::MX dy3 = vk * cs::MX::sin(x_temp(2));
                cs::MX dth3 = wk;
                cs::MX k3 = cs::MX::vertcat({dx3, dy3, dth3});

                x_temp = xk + dt * k3;
                cs::MX dx4 = vk * cs::MX::cos(x_temp(2));
                cs::MX dy4 = vk * cs::MX::sin(x_temp(2));
                cs::MX dth4 = wk;
                cs::MX k4 = cs::MX::vertcat({dx4, dy4, dth4});


                // Stato al passo successivo (discretizzazione RK4)
                cs::MX x_next = xk + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
                
                // Vincolo dinamico
                dyn = xk1 - x_next;

            }
            else{
                std::cout << "Error in the model type." << std::endl;
            }
            
        
            g.push_back(dyn);
            lbg = cs::DM::vertcat({lbg, cs::DM::zeros(nx)});
            ubg = cs::DM::vertcat({ubg, cs::DM::zeros(nx)});      


            // Obstacle avoidance
            for(int j=0; j<N_obs; j++){
                
                // Current obstacle position
                cs::MX obstacles_pos = p(cs::Slice(weights_start_idx + N_cost_params + nu + N_obs_info*j, weights_start_idx + N_cost_params + nu + N_obs_info*j +2));
                
                // Current obstacle velocity
                cs::MX obstacles_vel = p(cs::Slice(weights_start_idx + N_cost_params + nu + N_obs_info*j+2, weights_start_idx + N_cost_params + nu + N_obs_info*j +4));

                // Obstacle radius
                cs::MX obstacles_r = p(weights_start_idx + N_cost_params + nu + N_obs_info*j + 4);
                
                // Future obstacle position
                cs::MX fut_obstacles_pos = obstacles_pos + (k+1)*dt*obstacles_vel;
                // std::cout << "Obstacle dimension: " << obstacles_pos.size1() << std::endl;
           

                cs::MX obs_constr = cs::MX::sumsqr(xk1(cs::Slice(0,2)) - fut_obstacles_pos)
                    - (obstacles_r + r_robot)*(obstacles_r + r_robot);
                g.push_back(obs_constr);
                lbg = cs::DM::vertcat({lbg, cs::DM::zeros(1)});     // lower bound 0
                ubg = cs::DM::vertcat({ubg, cs::DM::ones(1)*1e20}); // upper bound +large


                
                // std::cout << "Obstacle constraint set" << std::endl;

            }
        }

        // Terminal constraint:
        cs::MX terminal_constr = cs::MX::sumsqr(x_rN(cs::Slice(0,2)) - xN(cs::Slice(0,2))) - s*s;
        g.push_back(terminal_constr);     // 1 slack variable, 
        lbg = cs::DM::vertcat({lbg, cs::DM::ones(ns)*-1e-3});       // lb < x_rN - xN - s
        ubg = cs::DM::vertcat({ubg, cs::DM::zeros(ns)});            // x_rN - xN -s < ub
        

        // **** CONSTRAINTS ON VARIABLES VALUES*****
        int nX = nx*(Np+1);
        int nU = nu*Np;
        int n_opt = nX + nU;           // dimensione di opt_vars

        // Optimization variables constraints
        lbx_full = cs::DM::ones(n_opt) * -1e20;
        ubx_full = cs::DM::ones(n_opt) *  1e20;

        // State constraints:
        for (int i = 0; i < nX; ++i) {
            lbx_full(i) = -1e20;
            ubx_full(i) =  1e20;
        }

        // Input constraints:
        for (int k = 0; k < Np; ++k) {
            lbx_full(nX + 2*k + 0) = v_min;
            ubx_full(nX + 2*k + 0) = v_max;
            lbx_full(nX + 2*k + 1) = w_min;
            ubx_full(nX + 2*k + 1) = w_max;
        }

        // Slack constraints:
        lbx_full = cs::DM::vertcat({lbx_full, cs::DM::zeros(ns)}); // s >= 0
        ubx_full = cs::DM::vertcat({ubx_full, cs::DM::ones(ns)*1e20}); // s non limitato superiormente
        

        for (int k = 0; k < Np; ++k) {
            cs::MX vk  = U(2*k + 0);
            cs::MX wk  = U(2*k + 1);

            if (k == 0) {

                // Extraction of the previous inputs
                cs::MX v_prev = p(weights_start_idx + N_cost_params + 0);  // aggiungi u_prev a p 
                cs::MX w_prev = p(weights_start_idx + N_cost_params + 1);

                g.push_back(vk - v_prev);
                lbg = cs::DM::vertcat({lbg, -delta_v_max});
                ubg = cs::DM::vertcat({ubg, delta_v_max});

                g.push_back(wk - w_prev);
                lbg = cs::DM::vertcat({lbg, -delta_w_max});
                ubg = cs::DM::vertcat({ubg, delta_w_max});
            } else {
                // Passi successivi: vincolo tra input consecutivi
                cs::MX v_prev = U(2*(k-1) + 0);
                cs::MX w_prev = U(2*(k-1) + 1);

                g.push_back(vk - v_prev);
                lbg = cs::DM::vertcat({lbg, -delta_v_max});
                ubg = cs::DM::vertcat({ubg, delta_v_max});

                g.push_back(wk - w_prev);
                lbg = cs::DM::vertcat({lbg, -delta_w_max});
                ubg = cs::DM::vertcat({ubg, delta_w_max});
            }        
        }

        // Variabili decisionali totali = X + U + s
        cs::MX opt_vars = cs::MX::vertcat(std::vector<cs::MX>{X,U,s});

        // NLP
        std::map<std::string, cs::MX> nlp = { {"x", opt_vars}, {"f", J}, {"g", cs::MX::vertcat(g)}, {"p", p} };

        // Solver setting
        cs::Dict opts;
        opts["ipopt.print_level"] = 0;
        opts["print_time"] = 0;
        opts["ipopt.tol"] = 1e-4;
        opts["ipopt.max_iter"] = 50;

        solver_ = nlpsol("solver", "ipopt", nlp, opts);

        U_previous = cs::DM::zeros(nu*Np);
        X_previous = cs::DM::zeros(nx*(Np+1));
        s_previous = cs::DM::zeros(ns);

        // ROS_INFO("DEBUG sizes: nX=%d, nU=%d, n_opt=%d, ng=%d", nX, nU, n_opt, ng);

    }


    void MpcPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
        if (!initialized_){
            nh_ = ros::NodeHandle(name);
            tf_ = tf;

            sub_odom = nh_.subscribe<nav_msgs::Odometry>("/locobot/odom", 1, &MpcPlanner::odomCallback, this);
            sub_obs = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &MpcPlanner::obstacleCallback, this);
            pub_cmd = nh_.advertise<geometry_msgs::Twist>("/locobot/cmd_vel", 1);
            pub_optimal_traj = nh_.advertise<nav_msgs::Path>("/locobot/move_base/TrajectoryPlannerROS/local_plan", 1);
            pub_ref_posearray = nh_.advertise<geometry_msgs::PoseArray>("/pose_array",1);

            loadParameters();
            
            buildSolver();
            

            initialized_ = true;
            ROS_INFO("MPC local planner initialized");
        }
        else{
            return;
        }
    }

    void MpcPlanner::loadParameters(){
        N_cost_params = 0;
        XmlRpc::XmlRpcValue q_list, r_list, p_list;

        if (nh_.getParam("/mpc_planner/Q_weights", q_list)) {
            int Q_size = q_list.size();

            if (Q_size != 3) {
                ROS_WARN("Expected 3 elements for Q, got %d", Q_size);
                return;
            }

            for (int i = 0; i < Q_size; ++i) {
                // Alcuni YAML parser li leggono come int -> serve cast esplicito
                double value = 0.0;
                if (q_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    value = static_cast<double>(q_list[i]);
                else if (q_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
                    value = static_cast<int>(q_list[i]);
                else
                    ROS_WARN("Unexpected type in Q[%d]", i);

                Q(i) = value;
            }
            N_cost_params = N_cost_params + Q_size;
            ROS_INFO("Q loaded");
        }
        else{
            ROS_ERROR("Error loading Q matrix weights");
        }

        if (nh_.getParam("/mpc_planner/R_weights", r_list)) {

            int R_size = r_list.size();
            if (R_size != 2) {
                ROS_WARN("Expected 2 elements for R, got %d", R_size);
                return;
            }

            for (int i = 0; i < R_size; ++i) {
                double value = 0.0;
                if (r_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    value = static_cast<double>(r_list[i]);
                else if (r_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
                    value = static_cast<int>(r_list[i]);
                R(i) = value;
            }
            N_cost_params = N_cost_params + R_size;
            ROS_INFO("R loaded");
        }
        else{
            ROS_ERROR("Error loading R matrix weights");
        }


        if (nh_.getParam("/mpc_planner/P_weights", p_list)) {

            int P_size = p_list.size();
            if (P_size != 3) {
                ROS_WARN("Expected 3 elements for P, got %d", P_size);
                return;
            }
        
            for (int i = 0; i < P_size; ++i) {
                double value = 0.0;
                if (p_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble)
                    value = static_cast<double>(p_list[i]);
                else if (p_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
                    value = static_cast<int>(p_list[i]);
                P(i) = value;
            }
            N_cost_params = N_cost_params + P_size;
            ROS_INFO("P loaded");
        }
        else{
            ROS_ERROR("Error loading P matrix weights");
        }

    }


    void MpcPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        /*
            Funzione di callback che viene eseguita quando il plugin si sottoscrive e riceve il messaggio di odometria.
        */

        current_odom_ = *msg;
    }

    void MpcPlanner::obstacleCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
        try{
            if (obstacles_list.size() != msg->name.size()-3){

                // necessità di ricreare il solver, qualcosa è cambiato.

                // ricreo il vettore degli ostacoli
                obstacles_list.clear();
                for (int i = 0; i < msg->name.size(); i++)
                {
                    std::string name = msg->name[i];
                    // filtra solo gli oggetti che ti interessano
                    if (name != "room" && name != "ground_plane" && name != "locobot")
                    {   
                        double radius = 0.25;


                        // STARE ATTENTI AL FATTO CHE ros::Time::now() FORNISCE IL TIME DATO DAL CLOCK O DEL PC NEL CASO 
                        // DI ROBOT REALE (usare "use_sim_real == "false"") O DELLA SIMULAZIONE GAZEBO (usare "use_sim_real == "true"")
                        ros::Time time = ros::Time::now();
                        Obstacle new_obstacle(msg->pose[i].position.x, msg->pose[i].position.y, radius, i, time);
                        obstacles_list.push_back(new_obstacle);
                    }
                }
                N_obs = obstacles_list.size();
                
                // ricreo il solver con le nuove informazioni sugli ostacoli
                buildSolver();
                // std::cout << "Solver built" << std::endl;


            }
            else{
                // non è necessario ricostruire il solver.

                // se l'obstacle_list è vuoto non ci sono ostacoli, quindi non serve fare nulla

                // se l'obstacle_list non è vuoto allora si devono aggiornare le posizioni degli oggetti
                // e le corrispondenti velocità
                if (!obstacles_list.empty()){
                    for (int i = 0; i < obstacles_list.size(); i++){
                        int index = obstacles_list[i].index;
                        ros::Time time = ros::Time::now();
                        obstacles_list[i].updateInfo(msg->pose[index].position.x, msg->pose[index].position.y, time);

                        // std::cout << "Velocity of the obstacle: " << obstacles_list[i].pos(0) << "  " << obstacles_list[i].pos(1)  << std::endl;
                        // std::cout << "Velocity of the obstacle: " << obstacles_list[i].vel(0) << "  " << obstacles_list[i].vel(1)  << std::endl;
                    }
                }

            }
            // std::cout << "Number of obstacles detected: " << N_obs << std::endl;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

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

        geometry_msgs::PoseStamped robot_pose_odom;
        robot_pose_odom.header.frame_id = "locobot/odom";
        robot_pose_odom.header.stamp = ros::Time(0); // latest available
        robot_pose_odom.pose = current_odom_.pose.pose;

        geometry_msgs::PoseStamped robot_pose_map;
        try {
            tf_->transform(robot_pose_odom, robot_pose_map, "map"); // trasforma in frame map
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform from locobot/odom to map failed: %s", ex.what());
            return false;
        }

        // ora robot_pose_map.pose.position contiene x,y in map frame
        double x_map = robot_pose_map.pose.position.x;
        double y_map = robot_pose_map.pose.position.y;
        double theta_map = tf2::getYaw(robot_pose_map.pose.orientation);


        // Extract odometry information from the "current_odom_" message
        double x  = x_map;
        double y = y_map;
        double theta = theta_map;


        // // Extract odometry information from the "current_odom_" message
        // double x = current_odom_.pose.pose.position.x;
        // double y = current_odom_.pose.pose.position.y;
        // double theta = tf2::getYaw(current_odom_.pose.pose.orientation);

        // std::cout << "Actual position of the robot: " << x << " " << y << std::endl;

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
            cmd_vel.linear.x = u_opt(0);
            cmd_vel.angular.z = u_opt(1);
            pub_cmd.publish(cmd_vel);
            goal_reached_=true;
        }
        else{
            std::cout << "------------- Distanza non raggiunta ----------------" << std::endl;

            cs::DM p = cs::DM::zeros(nx + nx*(Np+1) + N_cost_params + nu + N_obs*N_obs_info, 1);
            p(0) = x;
            p(1) = y; 
            p(2) = theta;

            buildReferenceTrajectory(p, Np, x, y);

            double Qx = Q(0);
            double Qy = Q(1);
            double Qth = Q(2);
            double Rv = R(0);
            double Rw = R(1);
            double Px = P(0);
            double Py = P(1);
            double Pth = P(2);

            int weights_start_idx = nx + nx*(Np+1);
            
            p(weights_start_idx + 0) = Qx;
            p(weights_start_idx + 1) = Qy;
            p(weights_start_idx + 2) = Qth;
            p(weights_start_idx + 3) = Rv;
            p(weights_start_idx + 4) = Rw;
            p(weights_start_idx + 5) = Px;
            p(weights_start_idx + 6) = Py;
            p(weights_start_idx + 7) = Pth;

            p(weights_start_idx + 8) = v;
            p(weights_start_idx + 9) = w;
            
            try
            {            
                if (N_obs != 0){
                    std::cout << "Dimension of obstacle_list: " << obstacles_list.size() << std::endl;
                    for (int i=0; i<N_obs; i++){
                        p(weights_start_idx + N_cost_params + nu + N_obs_info*i +0) = obstacles_list[i].pos(0);
                        p(weights_start_idx + N_cost_params + nu + N_obs_info*i +1) = obstacles_list[i].pos(1);
                        p(weights_start_idx + N_cost_params + nu + N_obs_info*i +2) = obstacles_list[i].vel(0);
                        p(weights_start_idx + N_cost_params + nu + N_obs_info*i +3) = obstacles_list[i].vel(1);
                        p(weights_start_idx + N_cost_params + nu + N_obs_info*i +4) = obstacles_list[i].r;

                        std::cout << "Position of the obstacle: " << obstacles_list[i].pos(0) << "  " << obstacles_list[i].pos(1)  << std::endl;
                        std::cout << "Velocity of the obstacle: " << obstacles_list[i].vel(0) << "  " << obstacles_list[i].vel(1)  << std::endl;
                        std::cout << "Radius of the obstacle: " << obstacles_list[i].r << std::endl;
                    }
                }
                std::cout << "obstacle inserted" << std::endl;
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            


            // prepara arg e chiama solver (warm-start se vuoi)
            cs::DM x0 = cs::DM::vertcat(std::vector<cs::DM>{X_previous, U_previous, s_previous});
            std::map<std::string, cs::DM> arg;
            arg["x0"] = x0;
            arg["lbx"] = lbx_full;
            arg["ubx"] = ubx_full;
            arg["lbg"] = lbg;
            arg["ubg"] = ubg;
            arg["p"]  = p;


            // Solve MPC optimization problem
            std::map<std::string, cs::DM> res = solver_(arg);
            if (res.count("x") == 0) {
                ROS_ERROR("Solver did not return 'x' in result map");
                return false; // o gestisci fallback
            }

            cs::DM solution;
            try {
                solution = res.at("x");
            } catch (const std::exception& e) {
                ROS_ERROR("Exception getting solution: %s", e.what());
                return false;
            }

            // auto stats = solver_.stats();
            // std::string status = static_cast<std::string>(stats["return_status"]);
            // if (status == "Solve_Succeeded" || status == "Feasible_Point_Found") {
            //     std::cout << "✅ Solver ha trovato una soluzione ammissibile.\n";
            // } else {
            //     std::cout << "❌ Solver non ha trovato una soluzione feasible. Stato: " << status << "\n";
            // }

            // std::cout << "Dimension of solution: " << (int)solution.size1() << std::endl;

            cs::DM X_opt = solution(cs::Slice(0, nx*(Np+1)));                // primi nx*(Np+1) valori → stati
            cs::DM U_opt = solution(cs::Slice(nx*(Np+1), nx*(Np+1)+nu*Np));  // restanti nu*Np valori → controlli
            cs::DM s_opt = solution(cs::Slice(nx*(Np+1) + nu*Np, nx*(Np+1) + nu*Np + 1));    

            // std::cout << "Dimension of X_opt: " << (int)X_opt.size1() << std::endl;
            // std::cout << "Dimension of U_opt: " << (int)U_opt.size1() << std::endl;

            // std::cout << "Final point of the optimal trajectory: " << X_opt(nx*(Np+1)-3) << " " << X_opt(nx*(Np+1)-2) << std::endl;
            // std::cout << "Final point of the reference trajectory: " << p(nx*(Np+1) + 0) << " " << p(nx*(Np+1) + 1) << std::endl;

            std::cout << "Dimension of p:" << (int)p.size1() << std::endl;
            // for(int i=0; i < (int)p.size1(); i++){
            //     std::cout << p(i) << std::endl;
            // }


            // for (int k=0;k<Np;++k){
            //     int base = nx*k;
            //     double th_pred = double(X_opt(base+2));
            //     int ref_base = nx + 3*k; // se p layout è stato fatto così
            //     double th_ref = double(p(ref_base+2));
            //     double diff = th_pred - th_ref;
            //     double diff_normalized = std::atan2(std::sin(th_pred - th_ref),std::cos(th_pred - th_ref));
            //     // wrapped difference
            //     double ang_err = angles::shortest_angular_distance(th_pred, th_ref);
            //     ROS_INFO("k=%d: th_pred=%f th_ref=%f diff=%f diff_normalized=%f ang_err=%f", k, th_pred, th_ref, diff, diff_normalized, ang_err);
            // }
          
            // Extracion of the first control input
            cmd_vel.linear.x = double(U_opt(0));
            cmd_vel.angular.z = double(U_opt(1));
            pub_cmd.publish(cmd_vel);

            // Shift di un passo sulla sequenza U
            for (int k = 0; k < Np-1; ++k) {
                U_previous(nu*k + 0) = U_opt(nu*(k+1) + 0);
                U_previous(nu*k + 1) = U_opt(nu*(k+1) + 1);
            }
            // ripeti l'ultimo
            U_previous(nu*(Np-1) + 0) = U_opt(nu*(Np-1) + 0);
            U_previous(nu*(Np-1) + 1) = U_opt(nu*(Np-1) + 1);

            for (int k = 0; k < Np; ++k) {
                X_previous(nx*k + 0) = X_opt(nx*(k+1) + 0);
                X_previous(nx*k + 1) = X_opt(nx*(k+1) + 1);
                X_previous(nx*k + 2) = X_opt(nx*(k+1) + 2);
            }
            // Ultimo stato = copia dell’ultimo (mantieni fermo)
            X_previous(nx*Np + 0) = X_opt(nx*Np + 0);
            X_previous(nx*Np + 1) = X_opt(nx*Np + 1);
            X_previous(nx*Np + 2) = X_opt(nx*Np + 2);

            
            // X_previous = X_opt;
            // U_previous = U_opt;
            s_previous = s_opt;
            
            // Show the generated optimized path
            nav_msgs::Path path_msg;
            path_msg.header.stamp = ros::Time::now();
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

            // std::cout << "Dimension of path_msg: " << (int)path_msg.poses.size() << std::endl;

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
        // Trova il punto della traiettoria più vicino al robot
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

        

        // Copia Np punti a partire dal closest_idx
        geometry_msgs::PoseArray ref_pose_array;
        for (int k = 0; k < Np_+1; ++k) {
            int traj_idx = closest_idx + k;

            if (traj_idx < global_plan_.size() - 1) {
                // punto normale (non ultimo)
                double x_ref = global_plan_[traj_idx].pose.position.x;
                double y_ref = global_plan_[traj_idx].pose.position.y;

                double dx = global_plan_[traj_idx+1].pose.position.x - x_ref;
                double dy = global_plan_[traj_idx+1].pose.position.y - y_ref;
                double theta_ref = std::atan2(dy, dx);

                p_(nx + nx*k + 0) = x_ref;
                p_(nx + nx*k + 1) = y_ref;
                p_(nx + nx*k + 2) = theta_ref;

                geometry_msgs::Pose pose;
                pose.position.x = x_ref;
                pose.position.y = y_ref;
                pose.position.z = 0.0;

                tf2::Quaternion q;
                q.setRPY(0, 0, theta_ref);
                pose.orientation = tf2::toMsg(q);
                ref_pose_array.poses.push_back(pose);
            }
            else {
                p_(nx + nx*k + 0) = goal_pos[0];
                p_(nx + nx*k + 1) = goal_pos[1];
                p_(nx + nx*k + 2) = goal_orient;

                geometry_msgs::Pose pose;
                pose.position.x = goal_pos[0];
                pose.position.y = goal_pos[1];
                pose.position.z = 0.0;

                tf2::Quaternion q;
                q.setRPY(0, 0, goal_orient);
                pose.orientation = tf2::toMsg(q);
                ref_pose_array.poses.push_back(pose);
            }
        }


        
        ref_pose_array.header.stamp = ros::Time::now();
        ref_pose_array.header.frame_id = "map";  // o il frame corretto per il tuo caso
        pub_ref_posearray.publish(ref_pose_array);

         // ---- PRINT per debug ----
        // std::cout << "=== Traiettoria di riferimento (Np punti) ===" << std::endl;
        // for (int k = 0; k < Np_; ++k) {
        //     std::cout << "Point " << k << ": x=" << p_(3 + 3*k + 0) << ", y=" << p_(3 + 3*k + 1) << ", theta=" << p_(3 + 3*k + 2) << std::endl;
        // }

        // std::cout << "=== Punto finale traiettoria di riferimento ===" << std::endl;
        // std::cout << "Point " << Np_-1 << ": x=" << p_(3 + 3*(Np-1) + 0) << ", y=" << p_(3 + 3*(Np_-1) + 1) << ", theta=" << p_(3 + 3*(Np_-1) + 2) << std::endl;

        // std::cout << "--- Goal finale ---" << std::endl;
        // const auto& goal = global_plan_.back();
        // double goal_x = goal.pose.position.x;
        // double goal_y = goal.pose.position.y;
        // double goal_yaw = tf2::getYaw(goal.pose.orientation);
        // std::cout << "Goal x=" << goal_x << ", y=" << goal_y << ", yaw=" << goal_yaw << std::endl;
        }

    // void MpcPlanner::loadParameters() {
    //     nh_.param("~control_dt", control_dt_, 0.1);
    //     nh_.param("~horizon", horizon_, 10);
    // }



} // namespace mpc_planner

void unwrapAngles(std::vector<double> &angles) {
    if (angles.empty()) return;
    for (size_t i = 1; i < angles.size(); ++i) {
        double diff = angles[i] - angles[i - 1];
        if (diff > M_PI)
            angles[i] -= 2.0 * M_PI;
        else if (diff < -M_PI)
            angles[i] += 2.0 * M_PI;
    }
}
