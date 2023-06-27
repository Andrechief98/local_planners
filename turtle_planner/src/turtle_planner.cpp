#include "turtle_planner.h"
#include <pluginlib/class_list_macros.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <cmath>
#include <boost/thread.hpp>
#include <iostream>



PLUGINLIB_EXPORT_CLASS(turtle_planner::TurtlePlanner, nav_core::BaseLocalPlanner)

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_pose_ = *msg;
  //ROS_INFO("Odometria ricevuta!");
}

//CALCOLO DISTANZA (norma del vettore differenza tra due vettori, vec1 e vec2)
double vect_norm2(std::vector<double> vec1, std::vector<double> vec2){
        double norma;
        std::vector<double> difference={0,0};
        
        for (int i=0; i<2; i++){
            difference[i]=vec1[i]-vec2[i];
        }
        
        norma = sqrt(difference[0]*difference[0]+difference[1]*difference[1]);
        return norma;
    }

//CALCOLO NORMA DI UN VETTORE
double vect_norm1(std::vector<double> vec1){
        double norma=0;
        norma = sqrt(vec1[0]*vec1[0]+vec1[1]*vec1[1]);
        return norma;
    }

//STABILISCE IL VERSORE DIREZIONE RIVOLTO DA vec1 A vec2
std::vector<double> compute_direction(std::vector<double> vec1, std::vector<double> vec2){
        
        std::vector<double> dir={0,0};
        double norm = vect_norm2(vec1, vec2);

        for(int i=0; i<2; i++){
            dir[i]=(vec1[i]-vec2[i])/norm;
        }

        return dir;
    }
/*
//CALCOLO ANGOLO INCLINAZIONE DI UN VETTORE (RISPETTO A UN ALTRO VETTORE)
std::vector<double> compute_inclination(std::vector<double> vec1, std::vector<double> vec2){
        
        double angle=0;
        double norm = vect_norm2(vec1, vec2);

        for(int i=0; i<2; i++){
            dir[i]=(vec1[i]-vec2[i])/norm;
        }

        return dir;
    }*/

//FUNZIONE SEGNO:
int sign(double expression){
    if (expression<1){
        return -1;
    }
    else if(expression>1){
        return 1;
    }
    else{
        return 0;
    }
}

//CALCOLA DIRETTAMENTE LA VELOCITA RISULTANTE SFRUTTANDO IL MODELLO DEL SOCIAL FORCE MODEL
std::vector<double> computeVelocityFromForce(std::vector<double> Ftot, std::vector<double> currentRobotVelocity, double maxVel, double dt){
    std::vector<double> new_vel={0,0};

    for(int i=0; i<Ftot.size(); i++){ 
        if(currentRobotVelocity[i]+dt*Ftot[i]>=0){
            if(std::fabs(currentRobotVelocity[i]+dt*Ftot[i])>=maxVel){
                new_vel[i]=maxVel;
            }
            else{
                new_vel[i]=currentRobotVelocity[i]+dt*Ftot[i];
            }
        }     
        else{
            if(std::fabs(currentRobotVelocity[i]+dt*Ftot[i])>=maxVel){
                new_vel[i]=-maxVel;
            }
            else{
                new_vel[i]=currentRobotVelocity[i]+dt*Ftot[i];
            }
        }      
    }

    return new_vel;
}


namespace turtle_planner{

    TurtlePlanner::TurtlePlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false){}

    TurtlePlanner::TurtlePlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        initialize(name, tf, costmap_ros);
    }

    TurtlePlanner::~TurtlePlanner() {}

    // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
    void TurtlePlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
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

    bool TurtlePlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
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

    bool TurtlePlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }


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

        //AGGIUNGERE RICEZIONE POSIZIONI OGGETTI PERSONE
        //
        //


        //CALCOLO DIREZIONE ROBOT-GOAL
        //1) creazione vettore coordinate robot:        
        curr_robot_coordinates={robot_pose_.pose.pose.position.x, robot_pose_.pose.pose.position.y}; //non so perchè con i messaggi di odometria non accetta il push_back() per inserirli in un vettore
        curr_robot_orientation=tf2::getYaw(robot_pose_.pose.pose.orientation);

        std::cout << "\n";
        std::cout << "**VETTORI COORDINATE GOAL E COORDINATE ROBOT CALCOLATI: " << std::endl;
        std::cout << "  *Goal coordinates vector  : " << goal_coordinates[0] << " " << goal_coordinates[1] << std::endl;
        std::cout << "  *Goal orientation z-axis (radians) [-pi,pi] : " << goal_orientation << std::endl;
        std::cout << "\n";
        std::cout << "  *Robot coordinates vector  : " << curr_robot_coordinates[0] << " " << curr_robot_coordinates[1] << std::endl;
        std::cout << "  *Robot orientation z-axis (radians) [-pi,pi] : " << curr_robot_orientation << std::endl; 

       

        //VERIFICA RAGGIUNGIMENTO GOAL E CALCOLO DELLE VELOCITÀ
        
        if(vect_norm2(goal_coordinates,curr_robot_coordinates)<=distance_tolerance){
            std::cout << "Distanza raggiunta" << std::endl;
            //coordinate raggiunte. Vettore velocità lineare rimane nullo.
            cmd_vel.linear.x=0.0;
            cmd_vel.linear.y=0.0;
            cmd_vel.linear.z=0.0;
           
            if(std::fabs(angles::shortest_angular_distance(curr_robot_orientation,goal_orientation))<=angle_tolerance){
                //anche l'orientazione del goal è stata raggiunta
                cmd_vel.angular.z=0.0;
                goal_reached=true;
                std::cout<<"Orientazione goal raggiunta"<<std::endl;
                std::cout<<"GOAL RAGGIUNTO"<<std::endl;
            }
            else{
                // Se le coordinate del goal sono state raggiunte ma l'orientazione finale no, la velocità angolare deve 
                // essere calcolata per far ruotare il robot nella posa finale del goal
                std::cout << "Orientazione non raggiunta" << std::endl;
                cmd_vel.angular.z=K_p*(angles::shortest_angular_distance(curr_robot_orientation,goal_orientation));
                cmd_vel.angular.y=0;
                cmd_vel.angular.x=0;
            }
        }
        else{
            std::cout << "Distanza non raggiunta" << std::endl;
            
            //IMPLEMENTAZIONE LEGGI DI CONTROLLO PROPORZIONALI VISTE NEL LAB03 ROS

            //verifichiamo che l'angolo tra direzione x del robot e il goal sia inferiore a un certo valore (pi/2)
            if(std::fabs(angles::shortest_angular_distance(curr_robot_orientation, std::atan2(goal_coordinates[1]-curr_robot_coordinates[1],goal_coordinates[0]-curr_robot_coordinates[0])))<=_PI/2){
                //possiamo eseguire una combo di traslazione lungo x e rotazione lungo z

                //VELOCITÀ ANGOLARE:
                //1) verifichiamo che la velocità angolare calcolata sia compresa tra [-1,1] rad/s
                if(std::fabs(K_p*(angles::shortest_angular_distance(curr_robot_orientation, std::atan2(goal_coordinates[1]-curr_robot_coordinates[1],goal_coordinates[0]-curr_robot_coordinates[0]))))<= 1){
                    cmd_vel.angular.z=K_p*(angles::shortest_angular_distance(curr_robot_orientation, std::atan2(goal_coordinates[1]-curr_robot_coordinates[1],goal_coordinates[0]-curr_robot_coordinates[0])));
                    cmd_vel.angular.y=0;
                    cmd_vel.angular.x=0;
                }
                else{
                    //se non lo è allora si verifica il segno della velocità e si setta la corrispondente velocità massima ammissibile
                    if(K_p*(angles::shortest_angular_distance(curr_robot_orientation, std::atan2(goal_coordinates[1]-curr_robot_coordinates[1],goal_coordinates[0]-curr_robot_coordinates[0])))>0){
                        cmd_vel.angular.z=1;
                        cmd_vel.angular.y=0;
                        cmd_vel.angular.x=0;
                    }
                    else{
                        cmd_vel.angular.z=-1;
                        cmd_vel.angular.y=0;
                        cmd_vel.angular.x=0;
                    }
                }

                //VELOCITÀ LINEARE:
                //si verifica che la velocità calcolata non superi un valore massimo di 0.5 m/s
                if(K_r*std::sqrt(pow(goal_coordinates[0]-curr_robot_coordinates[0],2)+pow(goal_coordinates[1]-curr_robot_coordinates[1],2))<=0.5){
                    cmd_vel.linear.x=K_r*std::sqrt(pow(goal_coordinates[0]-curr_robot_coordinates[0],2)+pow(goal_coordinates[1]-curr_robot_coordinates[1],2));
                    cmd_vel.linear.y=0;
                    cmd_vel.linear.z=0;
                }
                else{
                    cmd_vel.linear.x=0.5;
                    cmd_vel.linear.y=0;
                    cmd_vel.linear.z=0;
                }
            }
            else{
                //conviene prima ruotare il robot lungo z mantenendo la traslazione ferma
                if(angles::shortest_angular_distance(curr_robot_orientation, std::atan2(goal_coordinates[1]-curr_robot_coordinates[1],goal_coordinates[0]-curr_robot_coordinates[0]))>0){
                        cmd_vel.angular.z=1;
                        cmd_vel.angular.y=0;
                        cmd_vel.angular.x=0;
                    }
                    else{
                        cmd_vel.angular.z=-1;
                        cmd_vel.angular.y=0;
                        cmd_vel.angular.x=0;
                    }

            }
            
        }


        //PUBBLICAZIONE MESSAGGIO
        pub_cmd = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
        
        pub_cmd.publish(cmd_vel);

        std::cout << "\nmessaggio pubblicato" << std::endl;
        

        return true;
    }

    bool TurtlePlanner::isGoalReached()
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