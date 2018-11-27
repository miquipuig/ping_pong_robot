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
//#include <Eigen/Eigen>

//Eigen::MatrixXd eMap;

const double GOAL4METER=2; //Goals por cada metro real de distancia.
float map_resolution = 0;
int pixel_x = 0; //[pixeles de ancgura]
int pixel_y = 0; //[pixeles de altura]
double resolution =0; //Resolución del mapa [m/cell]
geometry_msgs::Point origin;
int PIXEL4GOAL=0;

int nrows = 50;
int ncols =4;

int determine_size()
{
    return 50;
}


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

      int x = pixel_x * map_resolution;
      int y = pixel_y * map_resolution;

      int size_x = x/2.2;
      int size_y = y/2;


 			//double *posicions= new double[nrows][4];

			int rows = 50, cols = 4;








      int x_pos[50] = {0, size_x, size_x, -size_x, -size_x, size_x, size_x, -size_x, -size_x, size_x, size_x, -size_x, -size_x, size_x, size_x, -size_x, -size_x, size_x, size_x, -size_x, -size_x, size_x, size_x, -size_x, -size_x};
      int y_pos[50] = {0, 0, -1, -1, -2, -2, -3, -3, -4, -4, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9, -10, -10, -11, -11, -12, -12, 12, 12, 11, 10, 10, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 0};
      int z_pos[50] = {0, -1, 1, -1, 0, -1, 1, -1, 0, -1, 1, -1, 0,-1, 1, -1, 0, -1, 1, -1, 0, -1, 1, -1, 0, -1, 1, -1, 0, -1, 1, -1, 0, -1, 1, -1, 0, -1, 1, -1, 0, -1, 1, -1, 0, -1, 1, -1, 0, -1};
      int w_pos[50] = {1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1};

      int lenght0f_Array = sizeof(x_pos) / sizeof(x_pos[0]);

	void chatterCallback(const nav_msgs::MapMetaData vector)
			{
				ROS_INFO("ENTRO");
				pixel_x = vector.width;//std::uint32_t a;
				pixel_y = vector.height;
				ROS_INFO("pixe_x: %d pixel_y %d", pixel_x, pixel_y);
			}



		  int main(int argc, char** argv){
				ros::init(argc, argv, "simple_navigation_goals");
				ros::NodeHandle n;

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
				 ROS_INFO("Llegamos");



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
                  //ROS_INFO("Valor %d en: %d", exploreGrid[i+j*grid_x], i*PIXEL4GOAL+j*pixel_x*PIXEL4GOAL );
              }
          }
          //Creamos la lista de goals budcando zeros en el GRID
          //Pendiente hacerlo mas inteligente
          //-------------------------------------
          ROS_INFO("Tenim un GRID de %d punts", grid_x*grid_y);
          double *goals = (double *) malloc(grid_x * grid_y *4 *sizeof(double));
          double zpos= 0;
          double wpos= 1;
          int numGoals=0;
          for(unsigned int i=0; i<grid_x;i++){
              //añadir cambiar xpos y wpos
              for(unsigned int j=0; j<grid_y;j++){
                //añadir cambiar xpos y wpos
                  if(exploreGrid[i+j*grid_x]==0){
                      //ROS_INFO("Entro");
                      //goals[numGoals*4]=(double)i/(double)GOAL4METER+(double)origin.x; //asignamos la X
                      //goals[numGoals*4+1]=(double)j/(double)GOAL4METER+ (double)origin.y;
                      goals[numGoals*4]= (double)i*(double)PIXEL4GOAL*resolution+ (double)origin.x;//asignamos la X
                      goals[numGoals*4+1]=(double)j*(double)PIXEL4GOAL*resolution+(double)origin.y;
                      goals[numGoals*4+2]=zpos;
                      goals[numGoals*4+3]=wpos;
                      numGoals++;
                      //ROS_INFO("GOAL %d %lf %lf %lf %lf",numGoals, (double)i/(double)GOAL4METER,(double)i/(double)GOAL4METER,zpos,wpos);
                  }

              }
          }


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









				//ros::Subscriber sub = n.subscribe("/map_metadata", 10,chatterCallback);




				//Create points matrix



        //tell the action client that we want to spin a thread by default

        MoveBaseClient ac("move_base", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(1.0))){
           ROS_INFO("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
        for( int i = 0; i < numGoals; i = i + 1 ) {

        goal.target_pose.pose.position.x =goals[i*4]; //(double)origin.x
        goal.target_pose.pose.position.y =goals[i*4+1];//(double)origin.y
        goal.target_pose.pose.orientation.w =goals[i*4+2];
        goal.target_pose.pose.orientation.z =goals[i*4+3];
          ROS_INFO("Sending goal");
	         ROS_INFO("Posicion %lf %lf %lf %lf", goals[i*4],goals[i*4+1], goals[i*4+2], goals[i*4+3]);
          ac.sendGoal(goal);

          ac.waitForResult();

         if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Goal aceptado, moviendose" );
	           else
            ROS_INFO("Error, goal inaceptable");
         }


      return 0;
}
