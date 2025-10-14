#ifndef _CLASSES_H_
#define _CLASSES_H_

#include <vector>
#include <string>
#include <gazebo_msgs/ModelStates.h>
#include <Eigen/Dense>



class Obstacle{
    public:
        Eigen::Vector2d pos{0, 0};
        Eigen::Vector2d vel{0, 0};
        Eigen::Vector3d bounding_box{0,0,0};
        double r = 0;

    //constructor
    Obstacle(double x, double y, double radius);

    
    void updateInfo(double x, double y, double dt);
};



#endif