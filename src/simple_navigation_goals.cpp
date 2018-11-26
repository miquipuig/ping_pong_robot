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


const double GOAL4METER=0.5; //Goals por cada metro real de distancia.
float map_resolution = 0;
int pixel_x = 0; //[pixeles de ancgura]
int pixel_y = 0; //[pixeles de altura]
double resolution =0; //Resolución del mapa [m/cell]
geometry_msgs::Point origin;
int PIXEL4GOAL=10;
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
 					ROS_INFO("Datos del mapa leidos2");
 					 mapaM = *grid;

 				 }

				 //cargamos el mapa en una matriz para poder trabajar mejor con el
				 int** mapa =new int*[pixel_x];
				 for(int i; i<pixel_x;i++){
 					mapa[i] = new int[pixel_y];
							for(int j;j<pixel_y;j++){
									mapa[i][j]=(int)mapaM.data[i+j*pixel_x];
							}
 				  }
					//definimos PIXEL4GOAL
					PIXEL4GOAL= (int)(GOAL4METER/resolution);
					ROS_INFO("Pixels/GOAL: %d",PIXEL4GOAL);
					//definimos una matriz con los Goals


				 /*for(int i; i<pixel_x*pixel_y; i++){
					 	mapa[i]=(int)mapaM.data[i];
						ROS_INFO("posició %d valod %d - ",i,mapa[i]);
				 }*/









				//ros::Subscriber sub = n.subscribe("/map_metadata", 10,chatterCallback);




				//Create points matrix

				int** matrix = new int*[rows];







        //tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(1.0))){
           ROS_INFO("Waiting for the move_base action server to come up");
        }

        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
        for( int i = 0; i < lenght0f_Array; i = i + 1 ) {

          goal.target_pose.pose.position.x = x_pos[i];
          goal.target_pose.pose.position.y = y_pos[i];
          goal.target_pose.pose.orientation.w = w_pos[i];
	  goal.target_pose.pose.orientation.z = z_pos[i];
            ROS_INFO("Sending goal");
	    ROS_INFO("Posicion %d%d%d%d", x_pos[i],y_pos[i], z_pos[i], w_pos[i]);
          ac.sendGoal(goal);

          ac.waitForResult();

         if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Goal aceptado, moviendose" );
	else
            ROS_INFO("Error, goal inaceptable");
         }


      return 0;
}
