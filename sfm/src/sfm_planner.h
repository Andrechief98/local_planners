#ifndef SFM_PLANNER_H_
#define SFM_PLANNER_H_

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <gazebo_msgs/ModelStates.h>

using namespace std;

static const double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static const double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;

nav_msgs::Odometry robot_pose_;
gazebo_msgs::ModelStates people_;

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

    void computeRepulsiveForce();

    void computeTotalForce();


private:
    ros::Publisher pub;
    ros::Subscriber sub;

    ros::Subscriber sub_odom;
    ros::Subscriber sub_people;
    ros::Subscriber sub_goal;
    ros::Publisher pub_cmd;
    ros::NodeHandle nh;

    ros::ServiceClient people_client;

    costmap_2d::Costmap2DROS* costmap_ros_;
    tf2_ros::Buffer* tf_;
    geometry_msgs::Twist cmd_vel_;
    bool initialized_;

    //GOAL:
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    geometry_msgs::PoseStamped goal_pose_;

    std::vector<double> goal_coordinates;
    double goal_orientation;

    //ROBOT ODOMETRY AND VELOCITY:
    std::vector<double> curr_robot_coordinates;
    double curr_robot_orientation;
    std::vector<double> curr_robot_lin_vel;
    double curr_robot_ang_vel;

    std::vector<double> new_robot_lin_vel;
    double new_robot_ang_vel_z;
    std::vector<double> new_robot_pos;


    //ROBOT SOCIAL FORCES
    std::vector<double> e;
    std::vector<double> F_att={0,0};
    std::vector<double> F_rep={0,0};
    std::vector<double> F_tot={0,0};

    double distance_tolerance=0.15;
    double angle_tolerance=0.13;
    bool goal_reached=false;
    double beta;
    
    double max_lin_acc_x=1;
    double max_angular_vel_z=1; //da ricavare dal file config dell'interbotix
    double desired_vel = 0.5; //valore da ricavare direttamente dal file dell'interbotix
    //double max_acc=2; //valore massimo di accelerazione (da sostituire con quelli ricavabili dal file di configurazione interbotix)
    double delta_t=0.2;
    double alfa=0.2;   //valore da inserire nel file di configurazione interbotix (se fattibile)
    double K_p=0.9; //costante proporzionale per il calcolo della velocità angolare (proporzionale all'errore);
    double K_r=0.9;
    };
};

#endif