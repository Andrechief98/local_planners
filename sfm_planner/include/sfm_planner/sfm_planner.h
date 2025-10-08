#ifndef SFM_PLANNER_H_
#define SFM_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <string>
#include <sfm_planner/classes.h>
#include <sensor_msgs/LaserScan.h>


using namespace std;

static const double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static const double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;

nav_msgs::Odometry robot_pose_;
//NECESSARI PER OTTENERE LE INFORMAZIONI SULL'OSTACOLO PIÙ VICINO DAL TOPIC /locobot/scan
sensor_msgs::LaserScan obstacle_distances_;
double obs_min_distance;
double angle_obs_min_distance;

//NECESSARI PER OTTENERE LE INFO DEI MODELLI DA GAZEBO
//gazebo_msgs::ModelStates people_;
std::vector<geometry_msgs::Pose> positions(10);
std::vector<std::string> stringa_vector(10);

Pedestrian global_model;

namespace sfm_planner{

class SfmPlanner : public nav_core::BaseLocalPlanner{
public:
    SfmPlanner();
    SfmPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    ~SfmPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    bool isGoalReached();

    void getOdometry();

    void set_Position_Orientation_Info();

    void setVelocityInfo();
    
    void computeAttractiveForce();

    void getPeopleInformation();

    void computePedestrianRepulsiveForce();

    void getObstacleInformation();

    void computeObstacleRepulsiveForce();

    void computeTotalForce();




private:
    ros::Publisher pub;
    ros::Subscriber sub;

    ros::Subscriber sub_odom;
    ros::Subscriber sub_people;
    ros::Subscriber sub_goal;
    ros::Subscriber sub_obs;
    ros::Publisher pub_cmd;
    ros::NodeHandle nh;

    ros::ServiceClient people_client;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;


    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_;
    geometry_msgs::Twist cmd_vel_;
    bool initialized_;

    //GOAL:
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    geometry_msgs::PoseStamped goal_pose_;

    std::vector<double> goal_coordinates;
    double goal_orientation;

    //MODELLI PEDONI GAZEBO
    std::vector<Pedestrian> pedestrian_list;


    //ROBOT ODOMETRY AND VELOCITY:
    std::vector<double> curr_robot_coordinates;
    double curr_robot_orientation;
    std::vector<double> curr_robot_lin_vel;
    double curr_robot_ang_vel;

    std::vector<double> new_robot_lin_vel;
    double new_robot_ang_vel_z;
    std::vector<double> new_robot_pos;

    //direzione attuale del robot
    std::vector<double> n_robot={0,0};


    //ROBOT SOCIAL FORCES
    std::vector<double> e;
    std::vector<double> F_att={0,0};
    std::vector<double> F_rep_ped={0,0};
    std::vector<double> F_rep_obs={0,0};
    std::vector<double> F_tot={0,0};

    double distance_tolerance=0.2;
    double angle_tolerance=0.13;
    bool goal_reached=false;
    //LEGGE DI CONTROLLO VELOCITÀ ANGOLARE
    double alfa_angle;
    double beta_angle;
    double theta_angle;
    double k=0.63;
    double h=0.3;
    double gam=0.2;

    double beta;
    
    double K_p=0.8; //costante proporzionale per il calcolo della velocità angolare (proporzionale all'errore);
    double max_lin_acc_x=2.5;
    double max_angular_vel_z=1.3; //da ricavare dal file config dell'interbotix
    double desired_vel = 0.5; //valore da ricavare direttamente dal file dell'interbotix
    double delta_t=0.2;

    //SOCIAL FORCE MODEL PARAMETERS
    double alfa=0.5;   //valore da inserire nel file di configurazione interbotix (se fattibile)
    double lambda=0.7;
    double A=0.99;
    double B=0.1;
    double radius=0.6;
    };
};

#endif
