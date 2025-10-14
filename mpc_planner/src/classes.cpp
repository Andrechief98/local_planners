#include <mpc_planner/classes.h>
#include <iostream>
#include <cmath>    //inclusione libreria matematica per eseguire radici quadrate
#include <cstdlib>
#include <gazebo_msgs/ModelStates.h>


#define DIMENSION 2 //dimensione del problema (due dimensioni, x e y)
#define DES_VEL 0.9312//valore di desired velocity (stesso valore sia per Vx che Vy)
#define LAMBDA 0.9928  //valore di lambda del SFM (articolo della prof)
#define TIME_STEP 0.2


Obstacle::Obstacle(double x, double y, double radius, int i){
        r=r;
        index = i;
        pos[0] = x;
        pos[1] = y;
    }

void Obstacle::updateInfo(double x, double y, double dt){
    vel = vel + (Eigen::Vector2d(x, y)-pos)/dt; 
    pos = Eigen::Vector2d(x, y);
    
}



