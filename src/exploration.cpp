#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <array>
#include <nav_msgs/MapMetaData.h>
#include <stdint.h>
#include <cstdint>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
//#include <Eigen/Eigen>
const double PI=3.14159265359;
//Eigen::MatrixXd eMap;

const double GOAL4METER=1; //Goals por cada metro real de distancia.
const int AREA_GRID_PX=10; //Area de exploración de obstáculo cerca del punto de grid
float map_resolution = 0;
int pixel_x = 0; //[pixeles de ancgura]
int pixel_y = 0; //[pixeles de altura]
double resolution =0; //Resolución del mapa [m/cell]
geometry_msgs::Point origin;
int PIXEL4GOAL=0;
int control=0;
int ind=0;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void chatterCallback(const std_msgs::String st)
{
    if(st.data =="EXPLORING"){
        control=4;
        //Le resto un goal pues suponemps que el anterior se ha cancelado
        if(ind>0){
        ind--;}
    }else
        control=0;

}




int main(int argc, char** argv){
      ros::init(argc, argv, "exploration");
      ros::NodeHandle n;
      ros::Subscriber sub = n.subscribe("state", 10,chatterCallback);
      ros::Publisher chatter_pub = n.advertise<std_msgs::String>("state", 1000);
      std_msgs::String msg;
      //cargamos los valores del mapa
      boost::shared_ptr<nav_msgs::MapMetaData const> sharedEdge;
      nav_msgs::MapMetaData edge;
      sharedEdge = ros::topic::waitForMessage<nav_msgs::MapMetaData>("/map_metadata",n);
      if(sharedEdge != NULL){
      	ROS_INFO("Datos del mapa leidos");
      	 edge = *sharedEdge;
      	 pixel_x=edge.width;
      	 pixel_y=edge.height;
      	 origin=edge.origin.position;
      	 resolution=edge.resolution;
       }

      boost::shared_ptr<nav_msgs::OccupancyGrid const> grid;
        nav_msgs::OccupancyGrid mapaM;
      	grid = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map",n);

      	if(sharedEdge != NULL){

      		 mapaM = *grid;

      	 }
      // Sección eliminada. Genera problemas en la memória y es innecesaria.
      //--------------------------------------------------------------------
      //cargamos el mapa en una matriz para poder trabajar mejor con él
      /*int** mapa = (int **)malloc(pixel_x*pixel_y*sizeof(int));
       //int** mapa =new int*[pixel_x];
       for(int i; i<pixel_x;i++){
      		mapa[i] = new int[pixel_y];
      		ROS_INFO("Fila consolidada");
      			for(int j;j<pixel_y;j++){

      					mapa[i][j]=(int)mapaM.data[i+j*pixel_x];
      			}
      	  }*/


      	//definimos PIXEL4GOAL
        //-------------------------------------
      	PIXEL4GOAL= (long)(1/(GOAL4METER*resolution));
        ROS_INFO("PIXEL4GOAL %d  ",PIXEL4GOAL);
        ROS_INFO("Resolution %lf",resolution);
      	//definimos un grid de exploración
        //-------------------------------------
       	int grid_x	=	(int)(pixel_x/PIXEL4GOAL);
      	int grid_y =(int)(pixel_y/PIXEL4GOAL);
        ROS_INFO("grid_x: %d", grid_x);
        ROS_INFO("grid_y: %d", grid_y);

        //Se tratará de una matriz con
        int *exploreGrid = (int *) malloc(grid_x * grid_y * sizeof(int));

        for(unsigned int i=0; i<grid_x;i++){
            for(unsigned int j=0; j<grid_y;j++){
                exploreGrid[i+j*grid_x]=(int)mapaM.data[i*PIXEL4GOAL+j*pixel_x*PIXEL4GOAL];
                //Si el valor del punto de gris es espacio conocido y libre ("0")
                // se revisa su alrededor par que no haya uns obstáculo cerca
                if((int)mapaM.data[i*PIXEL4GOAL+j*pixel_x*PIXEL4GOAL]==0){
                    //ROS_INFO("Punto agregado para explorar");
                    for(int n=0;n<=2*AREA_GRID_PX;n++){

                      for(int m=0;m<=2*AREA_GRID_PX;m++){

                        if((int)mapaM.data[i*PIXEL4GOAL+j*pixel_x*PIXEL4GOAL+n-AREA_GRID_PX+(m-AREA_GRID_PX)*pixel_x]==100){
                              //ROS_INFO("Punto no valido!");
                              exploreGrid[i+j*grid_x]=100;
                        }
                      }
                    }
                }
                //fin de revisar área de grid.
            }
        }
        //Creamos la lista de goals budcando zeros en el GRID
        //Pendiente hacerlo mas inteligente
        //-------------------------------------
        ROS_INFO("Tenim un GRID de %d punts", grid_x*grid_y);
        double *goals = (double *) malloc(grid_x * grid_y *4 *sizeof(double));
        double zpos= 1;
        double wpos= 1;
        int numGoals=0;
        int sentido=1;
        for(unsigned int i=0; i<grid_x;i++){
            //añadir cambiar xpos y wpos

            if(sentido==1){
                  //SUBO
                  zpos=1;
                  wpos=1;
                  for(unsigned int j=0; j<grid_y;j++){

                      if(exploreGrid[i+j*grid_x]==0){

                          goals[numGoals*4]= (double)i*(double)PIXEL4GOAL*resolution+ (double)origin.x;//asignamos la X
                          goals[numGoals*4+1]=(double)j*(double)PIXEL4GOAL*resolution+(double)origin.y;
                          goals[numGoals*4+2]=zpos;
                          goals[numGoals*4+3]=wpos;
                          numGoals++;

                      }
                }
                sentido =0;
            }else{
                  //BAJO
                  zpos=1;
                  wpos=-1;
                  for(int j=grid_y-1; j>=0;j--){

                      if(exploreGrid[i+j*grid_x]==0){

                          goals[numGoals*4]= (double)i*(double)PIXEL4GOAL*resolution+ (double)origin.x;//asignamos la X
                          goals[numGoals*4+1]=(double)j*(double)PIXEL4GOAL*resolution+(double)origin.y;
                          goals[numGoals*4+2]=zpos;
                          goals[numGoals*4+3]=wpos;
                          numGoals++;

                      }

                  }
                sentido=1;
            }
      }
        ROS_INFO("Se han definido %d puntos de exploracion.",numGoals);

         //Código que genera que supera el límite de memoria asignada.
        /*int** exploredGrid =(int **)malloc(grid_x*grid_y*sizeof(int));
      	//int** exploredGrid =new int*[grid_x];
      	ROS_INFO("grid_x %d , grid_y %d", grid_x,grid_y);
      	for(int k; k<grid_x;k++){
          //ROS_INFO("Entro");
      	 	exploredGrid[k] = new int[grid_y];
      			 for(int l;l<grid_y;l++){
                  ROS_INFO("Entro %d", l);
      				 		//revisamos si el punto es visitable
      					 //exploredGrid[k][l]=mapa[k*PIXEL4GOAL][l*PIXEL4GOAL];
      					 //exploredGrid[k][l]=0;
                 //ROS_INFO("Grid - (X: %d - Y: %d )",k,l);
      					 //ROS_INFO("Grid - (X: %d - Y: %d ): %d",k,l, exploredGrid[k][l]);

      			 }
      	 }*/

      	 //Creamos la lista de goals
      	 //possibles goals, aunqué realmente menos pues hay zonas no exploradas y ocupadas.

      	 /*
      	 int goalsnumber=grid_x*grid_y;
      	 double** goalslist=new double*[goalsnumber];
      	 for(int z; z<grid_x;z++){
      		 					goalslist[z] = new double[4];
      						goalslist[z][0]=0;
      						goalslist[z][1]=0;
      						goalslist[z][2]=0;
      						goalslist[z][3]=0;


      		 }*/

      MoveBaseClient ac("move_base", true);

      //wait for the action server to come up
      while(!ac.waitForServer(ros::Duration(1.0))){
         ROS_INFO("Waiting for the move_base action server to come up");
      }

      move_base_msgs::MoveBaseGoal goal;

      goal.target_pose.header.frame_id = "/map";
      goal.target_pose.header.stamp = ros::Time::now();

      ros::Rate loopRate(5);
      ind=0;
      while ( ros::ok() )
      {



        while (ind<numGoals && control==4){
        //for( int i = 0; i < numGoals; i++ ) {

            goal.target_pose.pose.position.x =goals[ind*4]; //(double)origin.x
            goal.target_pose.pose.position.y =goals[ind*4+1];//(double)origin.y
            goal.target_pose.pose.orientation.w =goals[ind*4+2];
            goal.target_pose.pose.orientation.z =goals[ind*4+3];
            ROS_INFO("Moviendose a GOAL %d de %d",ind+1, numGoals);
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Goal alcanzado");
            else
                ROS_INFO("Goal cancelado");
            ind++;
            if(ind==numGoals){
              ind==0;
              control=0;
              msg.data = "TRANSITION";
              chatter_pub.publish(msg);
            }
            ros::spinOnce();
         }
         ros::spinOnce();
         loopRate.sleep();
     }
     return 0;
}
