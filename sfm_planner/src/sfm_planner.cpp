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
    /*
        Funzione di callback che viene eseguita quando il plugin si sottoscrive e riceve il messaggio di odometria.
        Semplicemente salva in una variabile globale (robot_pose_) il contenuto del messaggio (altrimenti il
        messaggio potrebbe essere gestito solo all'interno di tale funzione) in modo che possa essere adoperata in 
        qualsiasi altra parte del programma.
    */

    robot_pose_ = *msg;
    //ROS_INFO("Odometria ricevuta!");
}

void people_callback(const gazebo_msgs::ModelStates::ConstPtr& people_msg)
{   
    /*
        Quando si considera la simulazione su Gazebo, è necessario ricavare le informazioni
        legate ai modelli dei pedoni presenti. Questa è una funzione di callback che viene eseguita quando
        ci si sottoscrive e si ricevono informazioni dal topic /model_states di Gazebo.    
    */

    /*ridefinisco la dimensione dei due vettori (variabili globali) sulla base di quanti modelli (pedoni, robot, muri, ecc ...) sono
    presenti nella simulazione Gazebo*/
    stringa_vector.resize(people_msg->name.size()); 
    positions.resize(people_msg->name.size());

    /*riempio i due vettori con i corrispondenti nomi e posizioni di ciascun modello presente in Gazebo. In questo modo, una volta che 
    il ciclo è finito, avremo le informazioni all'interno di variabili globali che possono essere adoperate anche in altre porzioni di
    codice del programma. Senza tale procedura le informazioni dovevano essere processate all'interno di tale funzione di callback*/
    for (int i=0; i<people_msg->name.size(); i++){
        stringa_vector[i]=people_msg->name[i];
        positions[i]=people_msg->pose[i]; 
    }

  
}

void obstacle_callback(const sensor_msgs::LaserScan::ConstPtr& obs_msg){
    /*
        Funzione di callback eseguita quando ci si sottoscrive e si ricevono informazioni dal topic del lidar del robot. 
        Il messaggio che si riceve avrà diverse informazioni. Quelle che più ci interessano corrispondono a:
            - range_min --> corrisponde alla minima distanza che il lidar riesce a misurare
            - range_max --> corrisponde alla massima distanza che il lidar riesce a misurare
            - ranges[] --> vettore di distanze ricavate dal lidar (sono distanze dalla posizione del lidar all'ostacolo rilevato)
            - angle_increment--> corrisponde all' incremento di angolo corrispondente ad ogni misura presente in ranges[] (vedere il ciclo per capire meglio).
    */

    //salvo le informazioni in una variabile globale nel caso siano utili in altre zone del programma
    obstacle_distances_ = *obs_msg;

    //RICAVO QUALE SIA LA DISTANZA E L'ANGOLAZIONE CORRISPONDENTE ALL'OGGETTO PIÙ VICINO:
    //setto un valore molto grande di distanza
    obs_min_distance=1000;

    for(int i=0; i<obstacle_distances_.ranges.size(); i++){
        //controllo sulla validità dei valori misurati dal lidar (se la distanza misurata è all'interno dell'intervallo [range_min; range_max])
        if(obstacle_distances_.ranges[i]>obstacle_distances_.range_min && obstacle_distances_.ranges[i]<obstacle_distances_.range_max)
            //se la misura è valida allora cerco quale sia il valore di distanza minore
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

//CLASSE LOCAL PLANNER (PLUGIN)
namespace sfm_planner{

    SfmPlanner::SfmPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false), pedestrian_list(10), listener(tfBuffer){}

    SfmPlanner::SfmPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false), listener(tfBuffer)
    {
        initialize(name, tf, costmap_ros);
    }

    SfmPlanner::~SfmPlanner() {}

    // Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
    void SfmPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {   
        /*
            Dovrebbe essere una funzione che viene eseguita nel momento in cui viene startata la simulazione e si avvia il move_base.
        */

        if(!initialized_)
        {   
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            initialized_ = true;
            
            goal_reached=false; //ho aggiunto solo questo flag utile per verificare se il goal è stato raggiunto, tutto il resto c'era già
            

        }

        initialized_=true;
        ROS_INFO("inizializzazione local planner avvenuta");
    }

    void SfmPlanner::getOdometry(){
        /*
            Metodo per ricavare l'odometria del robot. Semplicemente esegue la sottoscrizione al topic /locobot/odom e eseguirà la funzione di
            callback descritta in precedenza. L'odometria avrà le informazioni legate a:
                - posizione e orientazione (rispetto al frame "map")
                - velocità angolare e lineare (rispetto al frame "basefootprint", ovvero rispetto al robot stesso)  
        */

        sub_odom = nh.subscribe<nav_msgs::Odometry>("/locobot/odom", 1, &odom_callback);

        std::cout << "\n";
        std::cout << "**ODOMETRIA ROBOT RICEVUTA: " << std::endl;

        //Stampo a schermo le eventuali informazioni presenti nel messaggio dell'odometria
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
        /*
            dal messaggio dell'odometria (che viene salvato in una variabile globale - robot_pose_ - tramite la funzione corrispondente di 
            callback) vado a salvare le coordinate attuali del robot e la sua orientazione attuale (questa sarà espressa in quaternioni)
        */

        curr_robot_coordinates={robot_pose_.pose.pose.position.x, robot_pose_.pose.pose.position.y}; //non so perchè con i messaggi di odometria non accetta il push_back() per inserirli in un vettore
        curr_robot_orientation=tf2::getYaw(robot_pose_.pose.pose.orientation); //trasformo da quaternioni a radianti

        //Stampo le informazioni:
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
        /*
            Attraverso questa funzione estraggo semplicemente le informazioni contenute nel messaggio dell'odometria ma legate alla velocità 
            lineare e angolare. Una volta estratte però è necessario riferire tali velocità rispetto al frame "map" per poter poi fare in seguito
            i calcoli delle forze.
        */

        curr_robot_lin_vel={robot_pose_.twist.twist.linear.x, robot_pose_.twist.twist.linear.y}; //non so perchè con i messaggi di odometria non accetta il push_back() per inserirli in un vettore
        curr_robot_ang_vel=robot_pose_.twist.twist.angular.z;
        
        std::cout << "\n";
        std::cout << "**VETTORE VELOCITÀ ROBOT ATTUALE RISPETTO AL BASE_FOOTPRINT (frame del robot): " << std::endl;
        std::cout << "  *Robot velocity vector  : " << curr_robot_lin_vel[0] << " " << curr_robot_lin_vel[1] << std::endl; 

        //ESECUZIONE TRASFORMATA:
        //ci serve trasformare la velocità dal base_footprint al frame map

        ros::Rate rate(10.0);
        geometry_msgs::TransformStamped tf_result; //creo la variabile che contiene la trasformata (traslazione+rotazione)
        try {
            //salvo in "tf_result" la trasformata che mi porta dal frame "locobot/base_footprint" --> "map"
            tf_result = tfBuffer.lookupTransform("map", "locobot/base_footprint", ros::Time(0)); 
        } 
        catch (tf2::TransformException& ex) {
            //NON HO MESSO NULLA DA FARE NEL CASO IN CUI LA TRASFORMATA NON FUNZIONI

            // TODO: handle lookup failure appropriately
        }

        /*creo un quaternione che rappresenta la rotazione
         necessaria per passare da un frame all'altro. Tale quaternione deve essere uguale a 
         quello dato dalla trasformata relativa ai frame che stiamo considerando*/
        tf2::Quaternion q(
            tf_result.transform.rotation.x,
            tf_result.transform.rotation.y,
            tf_result.transform.rotation.z,
            tf_result.transform.rotation.w
        );

        /*creo un vettore (con dimensione 3) che rappresenta la traslazione
         necessaria per passare da un frame all'altro. In tal caso, non dovendo
         eseguire nessuna traslazione (visto che dobbiamo eseguire la trasformata 
         di una velocità), andiamo a creare una traslazione nulla*/
        tf2::Vector3 p(0,0,0);

        //creo un vettore trasformata
        tf2::Transform transform(q, p);

        //converto i vettori velocità da 2D a 3D (lungo z metto semplicemente 0)
        tf2::Vector3 velocity_in_child_frame(curr_robot_lin_vel[0],curr_robot_lin_vel[1],0);

        //eseguo la trasformata (usando il metodo precedente le nuove coordinate saranno ottenute semplicemente tramite moltiplicazione)
        tf2::Vector3 velocity_in_target_frame = transform * velocity_in_child_frame;

        //aggiorno il vettore velocità attuale del robot con i nuovi valori rispetto al frame "map"
        curr_robot_lin_vel={velocity_in_target_frame[0],velocity_in_target_frame[1]};

        //calcolo la direzione attuale del robot rispetto al frame "map" come versore della velocità attuale:
        for(int i=0; i<n_robot.size();i++){
            n_robot[i]=curr_robot_lin_vel[i]/vect_norm1(curr_robot_lin_vel);
        }

        //stampo informazioni
        std::cout << "VELOCITY NEL BASE LINK FRAME: " << std::endl;
        std::cout<< velocity_in_target_frame[0] << " " << velocity_in_target_frame[1]<< std::endl;
        std::cout<< curr_robot_lin_vel[0] << " " << curr_robot_lin_vel[1] << std::endl;


        return;
    }


    void SfmPlanner::computeAttractiveForce(){
        /*
            Calcolo della forza attrattiva secondo il SFM
        */
       
        F_att={0,0};
        e=compute_direction(goal_coordinates,curr_robot_coordinates);

        for(int i=0; i<e.size(); i++){
            //calcolo forza attrattiva
            F_att[i]=(desired_vel*e[i]-curr_robot_lin_vel[i])/alfa;
        }
        std::cout << "vettore e:"<< std::endl;
        std::cout << e[0] << " " << e[1] << std::endl;
        std::cout << "forza attrattiva goal calcolata" << std::endl;
        std::cout << F_att[0] << " " << F_att[1] << std::endl;

        return;
    }

    void SfmPlanner::getPeopleInformation(){
        /*
            Funzione che ricava le informazioni relative ai pedoni. In tal caso, visto che il plugin è quello della simulazione,
            le informazioni saranno ricavate direttamente da Gazebo tramite topic
        */

        //Pulisce il vettore che conterrà tutti gli oggetti "pedoni". In questo modo si ha un vettore vuoto da riempire nuovamente
        pedestrian_list.clear();
        
        /*Sottoscrizione al topic per ricevere le informazioni di TUTTI i modelli presenti in Gazebo. Attraverso la funzione di callback
          si ottengono direttamente "stringa_vector" (contiene tutti i nomi dei modelli) e "positions" (contiene posizione e orientazione 
          dei modelli). Tali informazioni sono necessarie per fare un filtro e ricavare solo quelli relativi ai pedoni
        */
        sub_people = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, &people_callback); //per le simulazioni su Gazebo

        /*RICERCA DI MODELLI I CUI NOMI PRESENTANO LA STRINGA "actor:"
         I modelli dei pedoni sono chiamati infatti "actor1", "actor2", ecc... 
         Attraverso tale procedimento andiamo quindi a ricavare il numero di pedoni presenti nella simulazione*/    
        int counter_actor=0;
        std::string string_to_find = "actor";
        
        for(int i=0; i<stringa_vector.size();i++){
            if(stringa_vector[i].find(string_to_find)!=-1){
                //se troviamo un modello che ha nel nome la stringa "actor" allora incremento il counter
                counter_actor+=1;
            }
            else{
                continue;
            }
        }

        //stampo la dimensione del vettore
        //std::cout << pedestrian_list.size() <<std::endl;

        //conoscendo il numero di pedoni presenti posso ridimensionare il vettore di oggetti
        pedestrian_list.resize(counter_actor);

        //creo ad ogni ciclo un oggetto "Pedestrian" e lo aggiungo alla lista pedoni
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

        /*PS. Si poteva fare tutto in un unico ciclo? Si... ma C++ funziona bene quando vuole lui per cui non andava correttamente
        adoperando .push_back() per aggiungere gli oggetti alla lista dei pedoni. Ergo ho fatto tutta sta roba qui 
        per poter conoscere il numero di pedoni che effettivamente ci sarebbero stati su Gazebo. Sebbene sia più lunga almeno funge*/
        
        std::cout << "PEDESTRIAN INFO RECEIVED" << std::endl;
        for (int i=0; i< pedestrian_list.size(); i++){
            std::cout << pedestrian_list[i].pedestrianName <<std::endl;
            std::cout << pedestrian_list[i].curr_pos[0] << " " << pedestrian_list[i].curr_pos[1] <<std::endl;
        }
        

        /*METODO PER RICAVARE LE INFORMAZIONI TRAMITE SERVIZIO AL POSTO DEI TOPIC--- NON CONSIDERARE QUESTE RIGHE*/
        
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
        /*
            Calcolo della forza repulsiva dei pedoni secondo il SFM
        */

        double dist=0;
        double F_fov=0;
        std::vector<double> n_ped={0,0};
        std::vector<double> F_r={0,0};
        F_rep_ped={0,0};

        for(int j=0; j<pedestrian_list.size(); j++){
            dist=vect_norm2(curr_robot_coordinates, pedestrian_list[j].curr_pos);
            n_ped=compute_direction(curr_robot_coordinates, pedestrian_list[j].curr_pos);
            F_fov=lambda+(1-lambda)*((1+compute_cos_gamma(n_robot,n_ped))/2); //n_robot=direzione corrente del robot

            //calcolo singola forza repulsiva del singolo pedone
            for(int k=0; k<F_rep_ped.size(); k++){
                F_r[k]=A*exp((pedestrian_list[j].radius+radius-dist)/B)*F_fov*n_ped[k]; 
            }

            //calcolo forza repulsiva risultante (tutti i pedoni)
            for(int k=0; k<F_rep_ped.size(); k++){
                F_rep_ped[k]=F_rep_ped[k]+F_r[k];
            }
        }   

        std::cout << "forza repulsiva pedoni calcolata" << std::endl;
        std::cout << F_rep_ped[0] << " " << F_rep_ped[1] << std::endl;

    }

    void SfmPlanner::getObstacleInformation(){
        /*
            Funzione che permette di prelevare le informazioni relative alla minima distanza misurata dal lidar
            e al corrispettivo angolo tramite la funzione di callback descritta in precedenza
        */
        sub_obs=nh.subscribe<sensor_msgs::LaserScan>("/locobot/scan", 1, &obstacle_callback);

        std::cout << "INFORMAZIONI OSTACOLO: " << std::endl;
        std::cout << "Min Distance= " << obs_min_distance << std::endl;
        std::cout << "Angle=  " << angle_obs_min_distance << std::endl;
    }


    void SfmPlanner::computeObstacleRepulsiveForce(){
        /*
            Calcolo della forza repulsiva dell'ostacolo più vicino secondo il SFM.
            Per poter però eseguire tale calcolo, è necessario eseguire la trasformata delle misure del lidar
            dal frame "locobot/laser_frame_link" al frame "map"
        */
        
        //il procedimento per eseguire la trasformata è lo stesso eseguito per le informazioni di velocità
        ros::Rate rate(10.0);
        geometry_msgs::TransformStamped tf_result;

        try {
            tf_result = tfBuffer.lookupTransform("map", "locobot/laser_frame_link", ros::Time(0));
        } 
        catch (tf2::TransformException& ex) {
            // TODO: handle lookup failure appropriately
        }

        //creo il quaternione che conterrà la ROTAZIONE relativa tra un frame e l'altro
        tf2::Quaternion q(
            tf_result.transform.rotation.x,
            tf_result.transform.rotation.y,
            tf_result.transform.rotation.z,
            tf_result.transform.rotation.w
        );

        /*creo il quaternione che conterrà la TRASLAZIONE relativa tra un frame e l'altro. In questo
        caso, visto che sono coordinate e non velocità, è necessario anche eseguire la traslazione*/
        tf2::Vector3 p(
            tf_result.transform.translation.x,
            tf_result.transform.translation.y,
            tf_result.transform.translation.z
        );
        
        //Creo la trasformata
        tf2::Transform transform(q, p);
        
        /*Esprimo le coordinate rispetto al frame "locobot/laser_frame_link" sfruttando le informazioni di distanza minima e angolo 
         corrispondente. Le coordinate sono espresse passando semplicemente da coordinate polari a coordinate cartesiane*/
        tf2::Vector3 point_in_child_coordinates(obs_min_distance*cos(angle_obs_min_distance),obs_min_distance*sin(angle_obs_min_distance),0);

        //Eseguo la trasformata ottenendo quindi le coordinate dell'ostacolo più vicino direttamente rispetto a "map"
        tf2::Vector3 point_in_parent_coordinates = transform * point_in_child_coordinates;

        std::cout << "INFORMAZIONI TRASFORMATA" << std::endl;
        std::cout << "Coordinate trasformate dal frame: " << tf_result.header.frame_id << " al frame: " << tf_result.child_frame_id << std::endl;
        std::cout << "Nuove coordinate rispetto al fixed frame: "<< point_in_parent_coordinates[0] << point_in_parent_coordinates[1] << std::endl;
       

        //CALCOLO EFFETTIVO DELLA FORZA REPULSIVA DELL'OSTACOLO
        double F_fov=0;
        std::vector<double> n_obs={0,0};
        F_rep_obs={0,0};

        n_obs=compute_direction(curr_robot_coordinates, {point_in_parent_coordinates[0],point_in_parent_coordinates[1]});  //AGGIUNGERE POSIZIONE DELL'OSTACOLO PIÙ VICINA RILEVATA DAL LIDAR
        F_fov=lambda+(1-lambda)*((1+compute_cos_gamma(n_robot,n_obs))/2); //forse al poste di e bisogna mettere la current robot orientation (dove punta l'asse x del robot)

        for(int k=0; k<F_rep_obs.size(); k++){
            F_rep_obs[k]=exp(1-(obs_min_distance/1))*F_fov*n_obs[k]; //forza del pedone espressa differentemente rispetto al robot
        }
           

        std::cout << "forza repulsiva ostacolo più vicino calcolata" << std::endl;
        std::cout << F_rep_obs[0] << " " << F_rep_obs[1] << std::endl;
    }


    void SfmPlanner::computeTotalForce(){
        /*
            Calcolo della forza totale che agisce sul robot
        */
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

        /*Istruzioni già presenti di default nel plugin. Semplicemente viene salvato all'interno di tale variabile globale
         "global_plan_" (vettore di geometry_msgs::PoseStamped già presente di default nel plugin) le coordinate dei vari 
          punti corrispondenti al GLOBAL PATH. A NOI INTERESSA COMUNQUE L'ULTIMO ELEMENTO DEL VETTORE, IL QUALE CONTERRÀ LE 
          INFORMAZIONI DEL GOAL*/
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        
        //quando settiamo un nuovo goal (planner frequency 0 Hz nel config file .yaml -> global planner chiamato una volta, solo all'inizio), 
        //resettiamo il flag. In questo modo in seguito potremo eseguire delle verifiche per capire se il goal è stato raggiunto
        goal_reached=false;

        //puliamo anche il vettore di coordinate che conteneva le coordinate del goal precedente
        goal_coordinates.clear();

        //Salviamo quindi solo l'ultimo punto del vettore che contiene tutto il global path
        int size_global_plan=global_plan_.size();
        goal_pose_=global_plan_[size_global_plan-1];

        //Setto le coordinate del goal all'interno di variabili globali che adopererò poi nel resto del programma:
        //goal_coordinates={goal_pose_.pose.position.x, goal_pose_.pose.position.y}; // corretto
        //goal_orientation=tf2::getYaw(goal_pose_.pose.orientation); //corretto
        
        std::cout << "COORDINATE GOAL RICEVUTE: " << std::endl;
        std::cout << "Pose Frame : " << goal_pose_.header.frame_id << std::endl; //FRAME = "map" (coincide con /locobot/odom)
        std::cout << "  Coordinates (meters) : " << goal_coordinates[0] << " " << goal_coordinates[1] << std::endl;
        std::cout << "  Orientation z-axis (radians) : " << goal_orientation << std::endl;

        return true;
    }


    bool SfmPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        /*
            FUNZIONE PRINCIPALE: È QUELLA DOVE VENGONO ESEGUITI TUTTI I CALCOLI EFFETTIVI
            Tale funzione verrà eseguita con una frequenza pari a "controller frequency" specificata nel file move_base_params.yaml (interbotix_xslocobot_nav/config)
        */

        if(!initialized_)
        {
            ROS_ERROR("This planner has not been initialized");
            return false;
        }

        //Adopero semplicemente le funzioni create in precedenza
        getOdometry();
        set_Position_Orientation_Info();
        setVelocityInfo();

        computeAttractiveForce();

        getPeopleInformation();     
        computePedestrianRepulsiveForce();

        getObstacleInformation();
        computeObstacleRepulsiveForce();

        computeTotalForce();

        /*A questo punto, una volta ottenuta la forza totale, posso procedere a determinare quale sarà la nuova linear velocity*/

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
                new_robot_lin_vel[i]=curr_robot_lin_vel[i]+delta_t*F_tot[i];// GROSSO PROBLEMA (SE F_tot È NEGATIVA ALLORA LA VELOCITÀ TENDE A RIDURSI A LIVELLO GLOBALE)
                
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
        /*
            Funzione già presente di default nel plugin.
            Viene eseguita prima di eseguire "computeVelocityCommands() per capire se il goal è stato raggiunto"
        */
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