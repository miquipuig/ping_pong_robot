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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//declaración del vector de la covarianza
std_msgs::Float64* covariance = new std_msgs::Float64[36]();
geometry_msgs::PoseWithCovariance pose;

const double C_LIMIT = 0.1;


void chatterCallback(const geometry_msgs::PoseWithCovarianceStamped vector)
{


double Cx = vector.pose.covariance[0];
double Cxy = vector.pose.covariance[1];
double Cy= vector.pose.covariance[6];
double Cyx= vector.pose.covariance[7];
//double Cy1= pose.covariance[36];

ROS_INFO("Cx: [%lf] - Cy: [%lf] - Cxy: [%lf] - Cyx: [%lf]", Cx, Cy,Cxy, Cyx);

if (Cx>C_LIMIT || Cy>C_LIMIT){
  ROS_INFO("Limite traspasado");
}
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "kidnapping");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  //declaración Subscriber
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("amcl_pose", 10,chatterCallback);
  //declaración Publisher
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("state", 1000);



  std_msgs::String msg;
  std::stringstream ss;
  ss << "hello world ";
  msg.data = ss.str();
  ros::Rate loop_rate(10);


  while (ros::ok())
  {


    chatter_pub.publish(msg);

    //ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    loop_rate.sleep();

  }
}
