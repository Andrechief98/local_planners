#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
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

#include <sfm_planner/sfm_planner.h>
#include <sfm_planner/functions.h>
#include <sfm_planner/classes.h>


PLUGINLIB_EXPORT_CLASS(sfm_planner::SfmPlanner, nav_core::BaseLocalPlanner)



// IMPLEMENTAZIONE EFFETTIVA PLUGIN

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_pose_ = *msg;
  //ROS_INFO("Odometria ricevuta!");
}

void people_callback(const gazebo_msgs::ModelStates::ConstPtr& people_msg)
{
    stringa_vector.resize(people_msg->name.size());
    positions.resize(people_msg->name.size());

    for (int i=0; i<people_msg->name.size(); i++){
        stringa_vector[i]=people_msg->name[i];
        positions[i]=people_msg->pose[i]; 
    }

    //il messaggio people_msg contiene:
    //      name --> vettore di stringhe, ogni elemento è il nome di ogni modello in Gazebo
    //      pose --> vettore di Pose msgs, ogni elemento è un messaggio Pose (legato al corrispondente elemento indicato nella stessa posizione nel vettore "name")
    //      twist --> vettore di Twist msgs, ogni elemento è un messaggio Twist (legato al corrispondente elemento indicato nella stessa posizione nel vettore "name")

    //dobbiamo estrarre il numero di pedoni presenti. 
    //I primi due elementi sono:
    //      GroundPlane
    //      TrossenRoboticsBuilding (dipende se definito nel world)

    //L'ultimo elemento è sempre:
    //      Locobot
  
}

void obstacle_callback(const sensor_msgs::LaserScan::ConstPtr& obs_msg){
    
    obstacle_distances_ = *obs_msg;
    obs_min_distance=1000;
    for(int i=0; i<obstacle_distances_.ranges.size(); i++){
        //controllo sulla validità dei valori misurati dal lidar
        if(obstacle_distances_.ranges[i]>obstacle_distances_.range_min && obstacle_distances_.ranges[i]<obstacle_distances_.range_max)
            if(obstacle_distances_.ranges[i] < obs_min_distance){
                //prelevo il valore più piccolo e conoscendo la sua posizione nel vettore, posso calcolare il corrispondente angolo partendo
                //dall'angolo minimo di rilevamento del LiDar
                obs_min_distance=obstacle_distances_.ranges[i];
                angle_obs_min_distance=obstacle_distances_.angle_min+i*obstacle_distances_.angle_increment;
            }

            else{
                continue;
            }

        else{
            continue;
        }
    }
    
}

namespace sfm_planner{

    SfmPlanner::SfmPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false), pedestrian_list(10),listener(tfBuffer){}

    SfmPlanner::SfmPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false), listener(tfBuffer)
    {
        initialize(name, tf, costmap_ros);
    }

    SfmPlanner::~SfmPlanner() {}

    // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
    void SfmPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if(!initialized_)
        {   
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            initialized_ = true;
            
            goal_reached=false; 
            

        }

        initialized_=true;
        ROS_INFO("inizializzazione local planner avvenuta");
    }

    void SfmPlanner::getOdometry(){
        //ODOMETRIA ROBOT:
        sub_odom = nh.subscribe<nav_msgs::Odometry>("/locobot/odom", 1, &odom_callback);

        std::cout << "\n";
        std::cout << "**ODOMETRIA ROBOT RICEVUTA: " << std::endl;
        /*
        std::cout << "  *Pose (coordinate+orientation) coordinate Frame  : " << robot_pose_.header.frame_id << std::endl; //FRAME = "map"
        std::cout << "      Coordinates (meters) : " << robot_pose_.pose.pose.position.x << " " << robot_pose_.pose.pose.position.y << std::endl;
        std::cout << "      Orientation z-axis (radians) : " << tf2::getYaw(robot_pose_.pose.pose.orientation) << std::endl;
        std::cout << "\n";
        std::cout << "  *Twist (velocity) coordinate Frame  : " << robot_pose_.child_frame_id << std::endl; //FRAME = "locobot/base_footprint"
        std::cout << "      *Linear Velocity (meters/second) : " << robot_pose_.twist.twist.linear.x << " " << robot_pose_.twist.twist.linear.y << std::endl;
        std::cout << "      *Angular Velocity z-axis (radians/second) : " << robot_pose_.twist.twist.angular.z << std::endl;
        */

       return;
    }

    void SfmPlanner::set_Position_Orientation_Info(){
        curr_robot_coordinates={robot_pose_.pose.pose.position.x, robot_pose_.pose.pose.position.y}; //non so perchè con i messaggi di odometria non accetta il push_back() per inserirli in un vettore
        curr_robot_orientation=tf2::getYaw(robot_pose_.pose.pose.orientation);

        std::cout << "\n";
        std::cout << "**VETTORI COORDINATE GOAL E COORDINATE ROBOT CALCOLATI: " << std::endl;
        std::cout << "  *Goal coordinates vector  : " << goal_coordinates[0] << " " << goal_coordinates[1] << std::endl;
        std::cout << "  *Goal orientation z-axis (radians) [-pi,pi] : " << goal_orientation << std::endl;
        std::cout << "\n";
        std::cout << "  *Robot coordinates vector  : " << curr_robot_coordinates[0] << " " << curr_robot_coordinates[1] << std::endl;
        std::cout << "  *Robot orientation z-axis (radians) [-pi,pi] : " << curr_robot_orientation << std::endl; 

        return;
    }

    void SfmPlanner::setVelocityInfo(){
        curr_robot_lin_vel={robot_pose_.twist.twist.linear.x, robot_pose_.twist.twist.linear.y}; //non so perchè con i messaggi di odometria non accetta il push_back() per inserirli in un vettore
        curr_robot_ang_vel=robot_pose_.twist.twist.angular.z;
        
        std::cout << "\n";
        std::cout << "**VETTORE VELOCITÀ ROBOT ATTUALE: " << std::endl;
        std::cout << "  *Robot velocity vector  : " << curr_robot_lin_vel[0] << " " << curr_robot_lin_vel[1] << std::endl; 

        return;
    }


    void SfmPlanner::computeAttractiveForce(){
        F_att={0,0};
        e=compute_direction(goal_coordinates,curr_robot_coordinates);

        for(int i=0; i<e.size(); i++){
            //calcolo forza attrattiva
            F_att[i]=(desired_vel*e[i]-curr_robot_lin_vel[i])/alfa;
        }

        std::cout << "forza attrattiva goal calcolata" << std::endl;
        std::cout << F_att[0] << " " << F_att[1] << std::endl;

        return;
    }

    void SfmPlanner::getPeopleInformation(){
        pedestrian_list.clear();
        
        sub_people = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &people_callback); //per le simulazioni su Gazebo

        //otteniamo "stringa_vector" (contiene tutti i nomi dei modelli) e "positions" (contiene posizione e orientazione dei modelli)
        int counter_actor=0;
        std::string string_to_find = "actor";
        
        for(int i=0; i<stringa_vector.size();i++){
            if(stringa_vector[i].find(string_to_find)!=-1){
                counter_actor+=1;
            }
            else{
                continue;
            }
        }

        std::cout << pedestrian_list.size() <<std::endl;
        pedestrian_list.resize(counter_actor);

        counter_actor=0;
        for(int i=0; i<stringa_vector.size();i++){
            if(stringa_vector[i].find(string_to_find)!=-1){
                Pedestrian modello;
                modello.setName(stringa_vector[i]);
                modello.setCurrentPosition(positions[i].position.x,positions[i].position.y);
                pedestrian_list[counter_actor]=modello;
                counter_actor+=1;
            }
            else{
                continue;
            }
        }

        
        std::cout << "PEDESTRIAN INFO RECEIVED" << std::endl;
        for (int i=0; i< pedestrian_list.size(); i++){
            std::cout << pedestrian_list[i].pedestrianName <<std::endl;
            std::cout << pedestrian_list[i].curr_pos[0] << " " << pedestrian_list[i].curr_pos[1] <<std::endl;
        }
        
        //RICEZIONE DELLE INFORMAZIONI DI UN DETERMINATO MODELLO SPECIFICATO SU GAZEBO (necessario andare a fornire il nome del modello)
        // people_client = nh.serviceClient <gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        // gazebo_msgs::GetModelState model_to_search;
        // model_to_search.request.model_name= (std::string) "actor1";
        
        // if(people_client.call(model_to_search))
        // {
        //     ROS_INFO("PR2's magic moving success!!");
        // }
        // else
        // {
        //     ROS_ERROR("Failed to magic move PR2! Error msg:%s",model_to_search.response.status_message.c_str());
        // }
        // std::cout << model_to_search.response.pose.position.x << std::endl;

        return;
    }


    void SfmPlanner::computePedestrianRepulsiveForce(){

        double dist=0;
        double F_fov=0;
        std::vector<double> n_ped={0,0};
        std::vector<double> F_r={0,0};
        F_rep_ped={0,0};

        for(int j=0; j<pedestrian_list.size(); j++){
            dist=vect_norm2(curr_robot_coordinates, pedestrian_list[j].curr_pos);
            n_ped=compute_direction(curr_robot_coordinates, pedestrian_list[j].curr_pos);
            F_fov=lambda+(1-lambda)*((1+compute_cos_gamma(e,n_ped))/2); //forse al poste di e bisogna mettere la current robot orientation (dove punta l'asse x del robot)

            for(int k=0; k<F_rep_ped.size(); k++){
                F_r[k]=A*exp((pedestrian_list[j].radius+radius-dist)/B)*F_fov*n_ped[k]; //forza del pedone espressa differentemente rispetto al robot
            }

            for(int k=0; k<F_rep_ped.size(); k++){
                F_rep_ped[k]=F_rep_ped[k]+F_r[k];
            }
        }   

        std::cout << "forza repulsiva pedoni calcolata" << std::endl;
        std::cout << F_rep_ped[0] << " " << F_rep_ped[1] << std::endl;

    }

    void SfmPlanner::getObstacleInformation(){

        sub_obs=nh.subscribe<sensor_msgs::LaserScan>("/locobot/scan", 1, &obstacle_callback);

        std::cout << "INFORMAZIONI OSTACOLO: " << std::endl;
        std::cout << "Min Distance= " << obs_min_distance << std::endl;
        std::cout << "Angle=  " << angle_obs_min_distance << std::endl;
    }


    void SfmPlanner::computeObstacleRepulsiveForce(){
        
        // tf2_ros::Buffer tfBuffer;
        // tf2_ros::TransformListener listener(tfBuffer);
        ros::Rate rate(10.0);
        geometry_msgs::TransformStamped tf_result;
        try {
        tf_result = tfBuffer.lookupTransform("map", "locobot/laser_frame_link", ros::Time(0));
        } catch (tf2::TransformException& ex) {
        // TODO: handle lookup failure appropriately
        }

        
        tf2::Quaternion q(
            tf_result.transform.rotation.x,
            tf_result.transform.rotation.y,
            tf_result.transform.rotation.z,
            tf_result.transform.rotation.w
        );
        tf2::Vector3 p(
            tf_result.transform.translation.x,
            tf_result.transform.translation.y,
            tf_result.transform.translation.z
        );

        tf2::Transform transform(q, p);
        tf2::Vector3 point_in_child_coordinates(obs_min_distance*cos(angle_obs_min_distance),obs_min_distance*sin(angle_obs_min_distance),0);
        tf2::Vector3 point_in_parent_coordinates = transform * point_in_child_coordinates;

        std::cout << "INFORMAZIONI TRASFORMATA" << std::endl;
        std::cout << "Coordinate trasformate dal frame: " << tf_result.header.frame_id << " al frame: " << tf_result.child_frame_id << std::endl;
        std::cout << "Nuove coordinate rispetto al fixed frame: "<< point_in_parent_coordinates[0] << point_in_parent_coordinates[1] << std::endl;
       

        double F_fov=0;
        std::vector<double> n_obs={0,0};
        F_rep_obs={0,0};

        n_obs=compute_direction(curr_robot_coordinates, {point_in_parent_coordinates[0],point_in_parent_coordinates[1]});  //AGGIUNGERE POSIZIONE DELL'OSTACOLO PIÙ VICINA RILEVATA DAL LIDAR
        F_fov=lambda+(1-lambda)*((1+compute_cos_gamma(e,n_obs))/2); //forse al poste di e bisogna mettere la current robot orientation (dove punta l'asse x del robot)

        for(int k=0; k<F_rep_obs.size(); k++){
            F_rep_obs[k]=exp(1.4-(obs_min_distance/0.8))*F_fov*n_obs[k]; //forza del pedone espressa differentemente rispetto al robot
        }
           

        std::cout << "forza repulsiva ostacolo più vicino calcolata" << std::endl;
        std::cout << F_rep_obs[0] << " " << F_rep_obs[1] << std::endl;
    }


    void SfmPlanner::computeTotalForce(){
        F_tot={0,0};
        for(int i=0; i<F_tot.size(); i++){

            F_tot[i]=F_att[i]+F_rep_ped[i]+F_rep_obs[i];   

            if(std::fabs(F_tot[i])>max_lin_acc_x){
                F_tot[i]=sign(F_tot[i])*max_lin_acc_x;
            }
        }

        std::cout << "\n";
        std::cout << "**FORZA RISULTANTE CALCOLATA: " << std::endl;
        std::cout << "  *Total force vector  : " << F_tot[0] << " " << F_tot[1] << std::endl; 
    }


    bool SfmPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        global_plan_.clear();
        global_plan_ = orig_global_plan;
        
        //quando settiamo un nuovo goal (planner frequency 0 Hz nel config file .yaml -> global planner chiamato una volta, solo all'inizio), 
        //resettiamo il flag per capire se il goal è stato raggiunto
        goal_reached=false;
        //puliamo anche il vettore di coordinate che conteneva le coordinate del goal precedente
        goal_coordinates.clear();

        //RICEZIONE GOAL MESSAGE
        int size_global_plan=global_plan_.size();
        goal_pose_=global_plan_[size_global_plan-1];

        goal_coordinates={goal_pose_.pose.position.x, goal_pose_.pose.position.y};
        goal_orientation=tf2::getYaw(goal_pose_.pose.orientation);
        
        std::cout << "COORDINATE GOAL RICEVUTE: " << std::endl;
        std::cout << "Pose Frame : " << goal_pose_.header.frame_id << std::endl; //FRAME = "map" (coincide con /locobot/odom)
        std::cout << "  Coordinates (meters) : " << goal_coordinates[0] << " " << goal_coordinates[1] << std::endl;
        std::cout << "  Orientation z-axis (radians) : " << goal_orientation << std::endl;

        
    
        return true;
    }


    bool SfmPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }


        getOdometry();
        set_Position_Orientation_Info();
        setVelocityInfo();

        computeAttractiveForce();

        getPeopleInformation();     
        computePedestrianRepulsiveForce();

        getObstacleInformation();
        computeObstacleRepulsiveForce();

        computeTotalForce();


        //CALCOLO NUOVA VELOCITA'
        new_robot_lin_vel={0,0};
        new_robot_pos={0,0};
        

        //VERIFICA RAGGIUNGIMENTO GOAL 
        
        if(vect_norm2(goal_coordinates,curr_robot_coordinates)<=distance_tolerance){
            std::cout << " ------------- Distanza raggiunta ----------------" << std::endl;
            //coordinate raggiunte. Vettore velocità lineare rimane nullo.
            new_robot_lin_vel={0,0};
            

           
            if(std::fabs(angles::shortest_angular_distance(curr_robot_orientation,goal_orientation))<=angle_tolerance){
                //anche l'orientazione del goal è stata raggiunta
                new_robot_ang_vel_z=0;
                goal_reached=true;
                std::cout<<"Orientazione goal raggiunta"<<std::endl;
                std::cout<<"GOAL RAGGIUNTO"<<std::endl;
            }
            else{
                // Se le coordinate del goal sono state raggiunte ma l'orientazione finale no, la velocità angolare deve 
                // essere calcolata per far ruotare il robot nella posa finale indicata
                std::cout << "Orientazione non raggiunta" << std::endl;
                new_robot_ang_vel_z=K_p*(angles::shortest_angular_distance(curr_robot_orientation,goal_orientation));
                }
            
        }
        else{
            std::cout << "------------- Distanza non raggiunta ----------------" << std::endl;

            for(int i=0; i<new_robot_lin_vel.size(); i++){
                new_robot_lin_vel[i]=curr_robot_lin_vel[i]+delta_t*F_tot[i];
                
                if(std::fabs(new_robot_lin_vel[i])>desired_vel){
                
                    new_robot_lin_vel[i]=sign(new_robot_lin_vel[i])*desired_vel;
                }

                new_robot_pos[i]=curr_robot_coordinates[i]+delta_t*new_robot_lin_vel[i]; //adoperiamo la velocità del modello (calcolata dalle forze) anzichè quella effettiva del robot
            }

            beta=std::atan2(new_robot_pos[1]-curr_robot_coordinates[1],new_robot_pos[0]-curr_robot_coordinates[0]);

            std::cout << "-------- INFO PER ANGOLI -------" << std::endl;
            std::cout << "  *beta= " << beta << std::endl;
            std::cout << "  *Robot orientation z-axis (radians) [-pi,pi] : " << curr_robot_orientation << std::endl; 
            std::cout << "  *Goal orientation z-axis (radians) [-pi,pi] : " << goal_orientation << std::endl;
            std::cout << "\n";
            std::cout << "-------- NUOVA VELOCITÀ ROBOT CALCOLATA --------- " << std::endl;
            std::cout << "  *New position : " << new_robot_pos[0] << " " << new_robot_pos[1] << std::endl;
            std::cout << "  *New velocity vector  : " << new_robot_lin_vel[0] << " " << new_robot_lin_vel[1] << std::endl; 
            std::cout << "  *New angular velocity : " << new_robot_ang_vel_z << std::endl;

            if(std::fabs(angles::shortest_angular_distance(curr_robot_orientation,beta))<=_PI/2){
                //la rotazione per muovere il robot nella direzione della forza è compresa in [-pi/2;pi/2]
                //possiamo eseguire una combo di rotazione e movimento in avanti

                //ROTAZIONE:
                new_robot_ang_vel_z=K_p*(angles::shortest_angular_distance(curr_robot_orientation, beta));

                if (std::fabs(new_robot_ang_vel_z)>max_angular_vel_z){
                    new_robot_ang_vel_z=sign(new_robot_ang_vel_z)*max_angular_vel_z;
                }

                //TRASLAZIONE GIA' CALCOLATA (RISPETTO AL FRAME "map")

            }

            else{
                //è preferibile far ruotare il robot verso la direzione della futura posizione prima di farlo muovere linearmente
                
                new_robot_lin_vel={0,0};
                new_robot_ang_vel_z=sign(angles::shortest_angular_distance(curr_robot_orientation,beta))*max_angular_vel_z;

            }

            // // new_robot_ang_vel_z=K_p*(angles::shortest_angular_distance(curr_robot_orientation, std::atan2(goal_coordinates[1]-curr_robot_coordinates[1],goal_coordinates[0]-curr_robot_coordinates[0])));
            
        }

        //NECESSARIO TRASFORMARE LA VELOCITÀ DA "map" AL FRAME DEL LOCOBOT "/locobot/base_link")
        ros::Rate rate(10.0);
        geometry_msgs::TransformStamped tf_result;
        try {
        tf_result = tfBuffer.lookupTransform("locobot/base_link", "map", ros::Time(0));
        } catch (tf2::TransformException& ex) {
        // TODO: handle lookup failure appropriately
        }

        
        tf2::Quaternion q(
            tf_result.transform.rotation.x,
            tf_result.transform.rotation.y,
            tf_result.transform.rotation.z,
            tf_result.transform.rotation.w
        );
        tf2::Vector3 p(0,0,0);

        tf2::Transform transform(q, p);
        tf2::Vector3 velocity_in_child_frame(new_robot_lin_vel[0],new_robot_lin_vel[1],0);
        tf2::Vector3 velocity_in_target_frame = transform * velocity_in_child_frame;

        std::cout << "VELOCITY NEL BASE LINK FRAME: " << std::endl;
        std::cout<< velocity_in_target_frame[0] << " " << velocity_in_target_frame[1]<< std::endl;


        //PUBBLICAZIONE MESSAGGIO
        pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);

        cmd_vel.angular.x=0.0;
        cmd_vel.angular.y=0.0;
        cmd_vel.angular.z=new_robot_ang_vel_z;
        // cmd_vel.linear.x=vect_norm1(new_robot_lin_vel);
        // cmd_vel.linear.y=0.0;
        cmd_vel.linear.x=std::fabs(velocity_in_target_frame[0]);
        cmd_vel.linear.y=0.0;
        cmd_vel.linear.z=0.0;

        pub_cmd.publish(cmd_vel);

        std::cout << "\nmessaggio pubblicato\n" << std::endl;
        std::cout << "\n-------------------------------------------------------------------\n\n\n" << std::endl;

        return true;
    }

    bool SfmPlanner::isGoalReached()
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }
        if(goal_reached){
            return true;
        }
        else{
            return false;
        }
        
    }
}