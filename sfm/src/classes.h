#ifndef _CLASSES_H_
#define _CLASSES_H_

#include <vector>


//******* DEFINIZIONE CLASSE GOAL ************
class Goal{
    public:
        int goalID;
        std::vector<double> coordinate={0,0}; //I VETTORI DI VETTORI SONO DA CONSIDERARE TUTTI SU UN'UNICA COLONNA

    //constructor
    Goal(int ID, double x_coordinate, double y_coordinate);
};


//******* DEFINIZIONE CLASSE OBJECT ************
class Object{
    public:
        std::vector<double> coordinate={0,0};
        double radius;

    //constructor
    Object(double x_coordinate, double y_coordinate, double r);
};


//******* DEFINIZIONE CLASSE PEDESTRIAN ************
class Pedestrian{
    private:
        std::vector<double> V_des={0,0}; //Desidered velocity 
        
        std::vector<double> e={0,0}; //desired direction
        std::vector<double> n_obs={0,0};
        std::vector<double> n_ped={0,0}; //pedestrians direction

        double alfa; //relaxation time
        double A; // strength repulsive force
        double B; // range repulsive force
        double radius; // raggio pedone
            
        
    
    public:
        int pedestrianID;
        std::vector<std::vector<double>> trajectory={{0,0,0,0,0}}; //single_point_of_the_trajectory=(x, y, Vx, Vy, t)
        std::vector<double> curr_pos={0,0}; //current position of Pedestrian
        std::vector<double> goal_position={0,0}; //goal position of Pedestrian

        std::vector<std::vector<double>> obs_position={{0,0}}; //obstacle position (ogni riga un nuovo ostacolo)
        std::vector<std::vector<double>> other_ped_positions={{0,0}}; //other pedestrian current position (ogni riga un nuovo pedone)

        std::vector<double> V_curr={0,0}; //current Velocity

        std::vector<double> F_goal={0,0}; //attractive Force (goal)
        std::vector<double> F_repul_obs={0,0}; //repulsive force (obstacle)
        std::vector<double> F_repul_ped={0,0}; //repulsive force (other pedestrian)
        std::vector<double> F_tot={0,0};

        bool g_reached=false;
    
    //DEFINIZIONE DELLE VARIE FUNZIONI CHE FARANNO PARTE DELLA CLASSE PEDONE

    Pedestrian(int pedID, double start_x, double start_y, double r);

    void setGoal(std::vector<double> goal_coordinate);

    void setCurrPosition(double x, double y);

    void setObsPositions(std::vector<Object> lista_ostacoli);

    void setOtherPedPositions(std::vector<Pedestrian> lista_pedoni);

    void compute_desired_velocity();  //calcolo della desired velocity 

    void computeAttractiveForce();

    void computeRepObsForce(std::vector<Object> lista_ostacoli);

    void computeRepPedForce(std::vector<Pedestrian> lista_pedoni);

    void computeTotalForce(); 

    void move_one_step(); //determina posizione e velocità del time step successivo

};


#endif