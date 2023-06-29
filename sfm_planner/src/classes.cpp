#include <sfm_planner/classes.h>
#include <iostream>
#include <cmath>    //inclusione libreria matematica per eseguire radici quadrate
#include <cstdlib>
#include <sfm_planner/functions.h>
#include <gazebo_msgs/ModelStates.h>


#define DIMENSION 2 //dimensione del problema (due dimensioni, x e y)
#define DES_VEL 0.9312//valore di desired velocity (stesso valore sia per Vx che Vy)
#define LAMBDA 0.9928  //valore di lambda del SFM (articolo della prof)
#define TIME_STEP 0.2


//DEFINIZIONI CLASSI:

Goal::Goal(int ID, double x_coordinate, double y_coordinate){
        goalID=ID;
        coordinate.clear();

        coordinate.push_back(x_coordinate);
        coordinate.push_back(y_coordinate);
    }


Object::Object(double x_coordinate, double y_coordinate, double r){
        radius=r;
        coordinate.clear();

        coordinate.push_back(x_coordinate);
        coordinate.push_back(y_coordinate);
    }


Pedestrian::Pedestrian(){}

void Pedestrian::setName(std::string name){
    pedestrianName=name;
}

void Pedestrian::setCurrentPosition(double x, double y){
    curr_pos={x,y};
}






// //*********** DEFINIZIONE CLASSE PEDESTRIAN (adoperata poi direttamente all'interno del file sfm.cpp) ****************

// //CONSTRUCTOR: 
// //se fosse definito adoperando solo "Pedestrian", il compiler la considerebbe come una semplice funzione.

// //Per poterla considerare come il constructor della classe "Pedestrian" dovremo adoperare la seguente sintassi:

// //                              Pedestrian::Pedestrian

// //In questo modo il compiler si rende conto che è la funzione di una classe

// Pedestrian::Pedestrian(int pedID, double start_x, double start_y, double r){
        
//         alfa=0.1854; //relaxation time
//         A=0.4611; // strength repulsive force
//         B=0.3; // range repulsive force
//         radius=r; //raggio pedone

//         //INIZIALIZZAZIONE COORDINATE PEDONE E GOAL
//         pedestrianID=pedID;

//         curr_pos.clear();
//         curr_pos.push_back(start_x);
//         curr_pos.push_back(start_y);

//     }

// //METHODS:
// //lo stesso discorso del constructor vale per i metodi della classe. Dovremo quindi adoperare la sintassi:

// //                      <method_type> Pedestrian::<method_name>
// void Pedestrian::setGoal(std::vector<double> coordinate){
//     goal_position.clear();

//     goal_position=coordinate;

//     return;
// }

// void Pedestrian::setCurrPosition(double x, double y){
//     curr_pos.clear();

//     curr_pos.push_back(x);
//     curr_pos.push_back(y);

//     return;
// }

// void Pedestrian::setObsPositions(std::vector<Object> lista_ostacoli){
//     obs_position.clear();

//     for(int i=0; i<lista_ostacoli.size(); i++){
//         obs_position.push_back(lista_ostacoli[i].coordinate);
//     }
    
//     return;

// }

// void Pedestrian::setOtherPedPositions(std::vector<Pedestrian> lista_pedoni){
//     other_ped_positions.clear();

//     for(int i=0; i<lista_pedoni.size(); i++){
//         other_ped_positions.push_back(lista_pedoni[i].curr_pos);
//     }
    
//     return;

// }

// void Pedestrian::compute_desired_velocity(){
//     e = compute_direction(goal_position, curr_pos); //versore direzione pedone - target
//     for(int i=0; i<DIMENSION; i++){
//         V_des[i]=e[i]*DES_VEL;
//     }
//     return;
// }

// void Pedestrian::computeAttractiveForce(){
//     compute_desired_velocity();
    
//     for(int i=0; i<DIMENSION; i++){
//         F_goal[i]=(V_des[i]-V_curr[i])/alfa;
//     }

//     std::cout << "forza attrattiva calcolata" << std::endl;

//     return;
// }

// void Pedestrian::computeRepObsForce(std::vector<Object> lista_ostacoli){
//     double dist=0;
//     double F_fov=0;
//     std::vector<double> F_r={0,0};
//     F_repul_obs={0,0};

//     for(int j=0; j<obs_position.size(); j++){
//         dist=vect_norm2(curr_pos, obs_position[j]);
//         n_obs=compute_direction(curr_pos, obs_position[j]);
//         F_fov=LAMBDA+(1-LAMBDA)*((1+compute_cos_gamma(e,n_obs))/2);

//         for(int k=0; k<DIMENSION; k++){
//             F_r[k]=exp(1.4-(dist/lista_ostacoli[j].radius))*F_fov*n_obs[k]; //forza dell'ostacolo espressa differentemente rispetto a quello del pedone
//         }

//         for(int k=0; k<DIMENSION; k++){
//             F_repul_obs[k]=F_repul_obs[k]+F_r[k];
//         }
//     }   

//     std::cout << "forza repulsiva ostacoli calcolata" << std::endl;

//     return;
// }


// void Pedestrian::computeRepPedForce(std::vector<Pedestrian> lista_pedoni){
//     double dist=0;
//     double F_fov=0;
//     std::vector<double> F_r={0,0};

//     F_repul_ped={0,0};

//     for(int j=0; j<other_ped_positions.size(); j++){
//         if(pedestrianID!=j+1){
//             dist=vect_norm2(curr_pos, other_ped_positions[j]);
//             n_ped=compute_direction(curr_pos, other_ped_positions[j]);
//             F_fov=LAMBDA+(1-LAMBDA)*((1+compute_cos_gamma(e,n_ped))/2);

//             for(int k=0; k<DIMENSION; k++){
//                 F_r[k]=A*exp((radius+lista_pedoni[j].radius-dist)/B)*F_fov*n_ped[k];
//             }
//         }
//         else{
//             //i calcoli sono eseguiti rispetto al pedone stesso, per cui sarà tutto nullo
//             F_r={0,0};
//         }

//         for(int k=0; k<DIMENSION; k++){
//             F_repul_ped[k]=F_repul_ped[k]+F_r[k];
//         }
//     }
//     std::cout << "forza repulsiva pedoni calcolata" << F_repul_ped[0] <<" " <<F_repul_ped[1] << std::endl;
    
//     return;
// }

// void Pedestrian::computeTotalForce(){

//     for(int i=0; i<DIMENSION;i++){
//         F_tot[i]=F_goal[i]+F_repul_obs[i]+F_repul_ped[i];
//     }

//     std::cout << "forza TOTALE calcolata "<<F_tot[0] << " " <<F_tot[1] << std::endl;

//     return;
// }

// void Pedestrian::move_one_step(){
//     for(int i=0; i<DIMENSION; i++){
//         if(std::fabs(V_curr[i])>=std::fabs(V_des[i])){
//             V_curr[i]=V_des[i];
//         }
//         if(g_reached){
//             V_curr[i]=0;
//         }
//         else{
//             curr_pos[i]=curr_pos[i]+V_curr[i]*TIME_STEP;
//             V_curr[i]=V_curr[i]+F_tot[i]*TIME_STEP;
//         }
//     }
    
//     return;
    
// }

