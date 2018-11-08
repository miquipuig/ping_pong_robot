#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <sstream>
#include "std_msgs/UInt16MultiArray.h"
#include <vector>
#include <cmath>

void chatterCallback(const geometry_msgs::PoseWithCovarianceStamped vector)
{

}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "kidnapping");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("amcl_pose", 10,chatterCallback);
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("state", 1000);

  ros::Rate loop_rate(10);

  std_msgs::String msg;
  std::stringstream ss;
  ss << "hello world ";
  msg.data = ss.str();

  while (ros::ok())
  {


    chatter_pub.publish(msg);

    //ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    loop_rate.sleep();

  }
}
