#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include <sstream>
#include "std_msgs/UInt16MultiArray.h"
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float64.h>
#include <stdbool.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
//Estados
const int STARTED	=0;
const int STOPPED	=1;
const int	BALL_APROX =2;
const int BALL_RECOLECT= 3;
const int EXPLORING= 4;
const int LOST =5;
const int HOME_RETURN =6;
int STATE=0;
int LAST_STATE=0;

//variables de aproximación a pelota
const double factorX=0.0004;
const double factorA=0.0008;
geometry_msgs::Vector3 aprox_vector;
geometry_msgs::Twist movetoball;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//declaración del vector de la covarianza
std_msgs::Float64* covariance = new std_msgs::Float64[36]();
geometry_msgs::PoseWithCovariance pose;


const double C_LIMIT = 0.09;
const double C_LIMIT_D = 0.05;

bool perdido =false;


move_base_msgs::MoveBaseGoal goal;

bool moves(move_base_msgs::MoveBaseGoal goals){
MoveBaseClient ac("move_base", true);
  goals.target_pose.header.stamp = ros::Time::now();
      //ROS_INFO("Enviar goal");

      while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }
      ac.sendGoal(goals);
      /*ac.waitForResult();
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
           return true;}
      else{
           return false;}*/
      //ROS_INFO("Goal enviado");
      return true;
}

void chatterCallback(const geometry_msgs::PoseWithCovarianceStamped vector)
{
    double Cx = vector.pose.covariance[0];
    double Cxy = vector.pose.covariance[1];
    double Cy= vector.pose.covariance[6];
    double Cyx= vector.pose.covariance[7];
    //double Cy1= ¡
    if (fabs(Cx)>C_LIMIT || fabs(Cy)>C_LIMIT || fabs(Cyx)>C_LIMIT ||fabs(Cxy)>C_LIMIT ){
      //ROS_INFO("Limite superado %lf", Cx );
      if(perdido==false){
        ROS_INFO("Me estoy perdiendo");
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.position.x = -0.4;
        goal.target_pose.pose.position.y = -0.4;
        goal.target_pose.pose.orientation.w = 1.5;

        ROS_INFO("Volviendo a casa");
        moves(goal);
        //ac.sendGoal(goal);
      }
       perdido=true;

    }else{
        //ROS_INFO("Limite no superado %lf", Cx);
      if(fabs(Cx)>C_LIMIT_D || fabs(Cy)>C_LIMIT_D || fabs(Cyx)>C_LIMIT_D ||fabs(Cxy)>C_LIMIT_D ){
        if(perdido==true){
          ROS_INFO("Ya se donde estoy");
          //parar
          goal.target_pose.header.frame_id = "base_link";
          goal.target_pose.pose.position.x = 0.0;
          goal.target_pose.pose.position.y = 0.0;
          goal.target_pose.pose.orientation.w = 1.0;
            ROS_INFO("Me paro");
          moves(goal);
          //ac.sendGoal(goal);
        }
        perdido=false;
      }
    }
}

void ballCallback(const geometry_msgs::Vector3& vector){

ROS_INFO("Pelotas?");
aprox_vector=vector;
  if (vector.z>0){
    LAST_STATE=STATE;
    aprox_vector=vector;
    STATE=BALL_APROX;
  }else{
    if(STATE==BALL_APROX){
      STATE=LAST_STATE;
    }
  }
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "control");

  //tell the action client that we want to spin a thread by default
/*MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }*/


  //declaración Subscriber
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("amcl_pose", 10,chatterCallback);
  ros::Subscriber dir = n.subscribe("/camera/direction", 10,ballCallback);
  //declaración Publisher
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("state", 1000);
  ros::Publisher ball_aprox = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1000);


  std_msgs::String msg;
  std::stringstream ss;
  ss << "hello world ";
  msg.data = ss.str();
  ros::Rate loop_rate(10);


  while (ros::ok())
  {
//ROS_INFO("looop");
//BALL_APROX
if(STATE==BALL_APROX){
  ROS_INFO("I like balls");

  movetoball.linear.x =aprox_vector.y*factorX;
  movetoball.angular.z=-aprox_vector.x*factorA;


ball_aprox.publish(movetoball);
}
    chatter_pub.publish(msg);

    //ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    loop_rate.sleep();

  }
}
