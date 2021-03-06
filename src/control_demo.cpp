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
#include <unistd.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include <nav_msgs/MapMetaData.h>
#include <kobuki_msgs/SensorState.h>
#include <stdint.h>
#include <cstdint>

//Estados
const int TRUE = 1;
const int FALSE	=0;
const int STARTED	=0;
const int STOPPED	=1;
const int	BALL_APPROACH =2;
const int BALL_RECOLECT= 3;
const int EXPLORING= 4;
const int LOST =5;
const int HOME_RETURN =6;
const int TRANSITION =7;
int STATE=1;
int LAST_STATE=0;
int CHANGE_STATE=0;
int BATTERY_LOW=FALSE;

//variables de aproximación a pelota
const double factorX=0.0004; //Factor de escalado direccional.
const double factorA=0.0008; //Factor de escalado angular
const double PRECISION=3; //precisión que se ha de cumplir para recojer la pelota. Mas pequeño mas precisión
const double MINIUM_X=0.003; // Mínimo factor de movimiento en X
const double MINIUM_Z=0.0060; // Mínimo factor de movimiento angular
geometry_msgs::Vector3 aprox_vector;
geometry_msgs::Twist movetoball;

//Posició braç
geometry_msgs::Point arm;
geometry_msgs::Point grip;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//declaración del vector de la covarianza
std_msgs::Float64* covariance = new std_msgs::Float64[36]();
geometry_msgs::PoseWithCovariance pose;


const double C_LIMIT = 0.09;
const double C_LIMIT_D = 0.08;

bool perdido =false;



void publishState(){
  ros::NodeHandle n;
  std_msgs::String msg;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("state", 1000);
  if(CHANGE_STATE==TRUE){
    if(STATE==STARTED){
        msg.data = "STARTED";
     chatter_pub.publish(msg);
    }else if(STATE==STOPPED){
      msg.data = "STOPPED";
     chatter_pub.publish(msg);
    }else if(STATE==BALL_APPROACH){
      msg.data =  "BALL_APPROACH";
     chatter_pub.publish(msg);
    }else if(STATE==BALL_RECOLECT){
     msg.data = "BALL_RECOLECT";
     chatter_pub.publish(msg);
    }else if(STATE==EXPLORING){
        msg.data =  "EXPLORING";
       chatter_pub.publish(msg);
    }else if(STATE==LOST){
        msg.data =  "LOST";
       chatter_pub.publish(msg);
    }else if(STATE==HOME_RETURN){
          msg.data =  "HOME_RETURN";
         chatter_pub.publish(msg);
    }

    CHANGE_STATE=FALSE;
  }

}




move_base_msgs::MoveBaseGoal goal;

bool moves(move_base_msgs::MoveBaseGoal goals){
  MoveBaseClient ac("move_base", true);

  goals.target_pose.header.stamp = ros::Time::now();
      //ROS_INFO("Enviar goal");

      while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }
      ac.sendGoal(goals);

      return true;
}

bool stop(){
  ROS_INFO("Me paro");
  MoveBaseClient ac("move_base", true);
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

      //ROS_INFO("Enviar goal");

      while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }
      // ac.sendGoal(goal);
      ac.cancelAllGoals();

      return true;
}
//Kidnapping
void chatterCallback(const geometry_msgs::PoseWithCovarianceStamped vector)
{
    double Cx = vector.pose.covariance[0];
    double Cxy = vector.pose.covariance[1];
    double Cy= vector.pose.covariance[6];
    double Cyx= vector.pose.covariance[7];
    //double Cy1= ¡
 if(STATE==EXPLORING||STATE==STARTED||STATE==LOST){
    //if ((fabs(Cx)>C_LIMIT && fabs(Cy)>C_LIMIT && fabs(Cyx)>C_LIMIT && fabs(Cxy)>C_LIMIT)&&perdido==false){
    if ((fabs(Cx)>C_LIMIT && fabs(Cy)>C_LIMIT)&&perdido==false){
      //ROS_INFO("Limite superado %lf", Cx );
        if(STATE==EXPLORING||STATE==STARTED){
          LAST_STATE=STATE;
        }
        STATE=LOST;
        CHANGE_STATE=TRUE;
        publishState();
        stop();
        ROS_INFO("Me estoy perdiendo, vuelvo a casa");

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.pose.position.x = -0.4;
        goal.target_pose.pose.position.y = -0.4;
        goal.target_pose.pose.orientation.w = 1.0;


        moves(goal);
        perdido=true;
        //ac.sendGoal(goal);

    //}else if((perdido==true)&&(fabs(Cx)<C_LIMIT_D || fabs(Cy)<C_LIMIT_D || fabs(Cyx)<C_LIMIT_D ||fabs(Cxy)<C_LIMIT_D )){
      }else if((perdido==true)&&(fabs(Cx)<C_LIMIT_D || fabs(Cy)<C_LIMIT_D)){

          STATE=LAST_STATE;
            ROS_INFO("Ya se donde estoy. Continuo. %d", LAST_STATE);
          CHANGE_STATE=TRUE;

          publishState();
          perdido=false;
          //ac.sendGoal(goal);



    }
  }
}

//Ball Recolection detects
void ballCallback(const geometry_msgs::Vector3& vector){
  aprox_vector=vector;

  if (vector.z>0){
  aprox_vector=vector;
      //ROS_INFO("estado: %d",STATE);
      if(STATE==EXPLORING||STATE==STARTED){
        //ROS_INFO("dentro: %d",STATE);
        LAST_STATE=STATE;
        STATE=BALL_APPROACH;
        CHANGE_STATE=TRUE;
        publishState();
        stop();
      }
  }else if(STATE==BALL_APPROACH){

      STATE=LAST_STATE;
      CHANGE_STATE=TRUE;
      publishState();

      //ACCIONES DE BRAZO MECÁNICO AQUÍ
  }
}

bool return_home(){
  ROS_INFO("Volviendo a casa");
  MoveBaseClient ac("move_base", true);
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  goal.target_pose.pose.orientation.z = 0.0;

     while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("cacacacac");
      }
      ac.sendGoal(goal);

      return true;
}

void stateCallback(const std_msgs::String state)
{
    if(state.data =="EXPLORING"){
          LAST_STATE=STATE;
          STATE=EXPLORING;
    }else if(state.data =="TRANSITION"){
          STATE=LAST_STATE;
          CHANGE_STATE=TRUE;
          stop();
    }else if(state.data =="STOPPED"){
            STATE=STOPPED;
            stop();

    }else if(state.data =="HOME_RETURN"){
            LAST_STATE=STATE;
            STATE=HOME_RETURN;
            publishState();
            stop();
            return_home();

    }else if(state.data =="STARTED"){
            LAST_STATE=STARTED;
            STATE=STARTED;
            publishState();

    }else if(state.data =="BALL_RECOLECT"){
            if(STATE!=BALL_RECOLECT){
                LAST_STATE=STATE;
            }
            STATE=BALL_RECOLECT;
            publishState();

    }
}

void batteryCallback(const kobuki_msgs::SensorState state)
{
      int battery=state.battery;
      //ROS_INFO("Bateria %d",battery);
      if(BATTERY_LOW==FALSE&&battery<100){
            if(STATE==EXPLORING||STATE==STARTED){
              LAST_STATE=STATE;
            }else{
              LAST_STATE=STOPPED;
            }
            ROS_INFO("Voy a recargar!");
            STATE=HOME_RETURN;
            CHANGE_STATE=TRUE;
            publishState();
            return_home();
            BATTERY_LOW=TRUE;

      }
      else if(BATTERY_LOW==TRUE&&battery>140){
        ROS_INFO("Bateria cargada");
        BATTERY_LOW=FALSE;
        STATE=LAST_STATE;
        publishState();
      }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "control");

  //declaración Subscriber
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("amcl_pose", 10,chatterCallback);
  ros::Subscriber dir = n.subscribe("/camera/direction", 10,ballCallback);
  ros::Subscriber sta = n.subscribe("state", 10, stateCallback);
  ros::Subscriber bat = n.subscribe("/mobile_base/sensors/core", 10, batteryCallback);

  //declaración Publisher STATE
  std_msgs::String msg;
  //std::stringstream ss;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("state", 1000);
  //declaración Publisher TELEOP
  ros::Publisher ball_approach = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

  ros::Publisher armPub = n.advertise<geometry_msgs::Point>("/topic_coordenades_objecte", 1000);

  ros::Publisher gripPub = n.advertise<geometry_msgs::Point>("/topic_coordenades_pinca", 1000);

  ros::Rate loop_rate(50);
  int count=0;
  int approachCount=0;
  while (ros::ok())
  {
    if (count<20){
        count++;
    }else{
      //ROS_INFO("Envio estado");
      arm.x=4;
      arm.y=0;
      arm.z=0;
      armPub.publish(arm);
      count=0;
      grip.x=50;
      gripPub.publish(grip);

  }
  //CANCELAR ACCIONES DE move_base
  /*if(CANCEL_GOAL==TRUE){
    stop();
    CANCEL_GOAL==FALSE;
  }*/

//------------------------------
//BALL_APPROACH
//------------------------------
if (approachCount==0){
      if(STATE==BALL_APPROACH){
        //ROS_INFO("I like balls");
        //ROS_INFO("1 - Canvi estat a BALL_RECOLECT? %d",STATE );
        double xb = aprox_vector.y*factorX;
        double zb = -aprox_vector.x*factorA;
        //ROS_INFO ("INI xb: %lf  zb: %lf", xb,zb);
        if(fabs(xb)<MINIUM_X){
          //ROS_INFO ("MID abs(xb): %lf  MINIUM_X: %lf", abs(xb),MINIUM_X);
          //ROS_INFO("Minimo X");
          if(xb>0){
              //ROS_INFO("Minimo X +");
              xb=MINIUM_X;
          }else if(xb<0){
            //ROS_INFO("Minimo X -");
            xb=-MINIUM_X;
          }
        }

        if(fabs(zb)<MINIUM_Z){
            //ROS_INFO("Minimo Z");
          if(zb>0){
              //ROS_INFO("Minimo Z +");
            zb=MINIUM_Z;
          }else if(zb<0){
            //ROS_INFO("Minimo Z -");
            zb=-MINIUM_Z;
          }
        }
        //ROS_INFO ("Movimiento en x: %lf  Angular: %lf", xb,zb);

        movetoball.linear.x =xb;
        movetoball.angular.z=zb;

        ball_approach.publish(movetoball);

        if(abs(aprox_vector.y)<=PRECISION && abs(aprox_vector.x)<=PRECISION){
          STATE=BALL_RECOLECT;
          CHANGE_STATE=TRUE;
          publishState();
          //ROS_INFO("2 - Canvi estat a BALL_RECOLECT? %d",STATE );
        }
      }

      //------------------------------
      //BALL_RECOLECT
      //------------------------------
      else if(STATE==BALL_RECOLECT){


      stop();
      ROS_INFO("Recollida de pilota");

      arm.x=4;
      arm.y=0;
      arm.z=0;
      grip.x=20;

      armPub.publish(arm);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      ROS_INFO("1.-Posicio inicial.");


      usleep(1000000);

      arm.x=10;
      arm.y=0;
      arm.z=0;
      grip.x=20;

      armPub.publish(arm);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);

      ROS_INFO("2.-Posicio en U.");

      usleep(2000000);
      arm.x=19;
      arm.y=0;
      arm.z=-11.5;
      grip.x=20;

      armPub.publish(arm);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);

      ROS_INFO("3.-Posicio final oberta.");
      usleep(2000000);
      arm.x=19;
      arm.y=0;
      arm.z=-11.5;
      grip.x=20;

      armPub.publish(arm);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);

      ROS_INFO("3.-Posicio final oberta.");
      usleep(500000);
      arm.x=19;
      arm.y=0;
      arm.z=-11.5;
      grip.x=30;

      armPub.publish(arm);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      ROS_INFO("4.-Posicio final 30.");


      usleep(500000);
      arm.x=19;
      arm.y=0;
      arm.z=-11.5;
      grip.x=38;

      armPub.publish(arm);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);

      ROS_INFO("5.-Posicio final 38.");
      usleep(500000);
      arm.x=19;
      arm.y=0;
      arm.z=-11.5;
      grip.x=50;

      armPub.publish(arm);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);
      usleep(100000);
      gripPub.publish(grip);

      ROS_INFO("6.-Posicio final tancada.");

      usleep(2000000);
      arm.x=19;
      arm.y=0;
      arm.z=-8;
      grip.x=50;

      armPub.publish(arm);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);

      ROS_INFO("6.-Posicio de retorn.");
      usleep(2000000);

      arm.x=19;
      arm.y=0;
      arm.z=-5;
      grip.x=50;

      armPub.publish(arm);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);

      ROS_INFO("6.-Posicio de retorn.");
      usleep(2000000);

      arm.x=10;
      arm.y=0;
      arm.z=0;
      grip.x=50;

      armPub.publish(arm);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);


      ROS_INFO("6.-Posicio de retorn en U.");
      usleep(1000000);

      arm.x=10;
      arm.y=0;
      arm.z=0;
      grip.x=50;

      armPub.publish(arm);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);


      ROS_INFO("6.-Posicio de retorn en U.");

      usleep(1000000);

      arm.x=4;
      arm.y=0;
      arm.z=0;
      grip.x=50;

      armPub.publish(arm);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);
      usleep(10000);
      gripPub.publish(grip);

      ROS_INFO("8.-Posicio de inicial tancada.");
      usleep(2000000);




      STATE=LAST_STATE;
      CHANGE_STATE=TRUE;
      publishState();
      approachCount=700;

      ROS_INFO("Volviendo al estado anterior: %d", STATE);
      }

}else{
approachCount--;


}





//------------------------------
//Enviar el último estado
//------------------------------
if(CHANGE_STATE==TRUE){
  if(STATE==STARTED){
      msg.data = "STARTED";
   chatter_pub.publish(msg);
  }else if(STATE==STOPPED){
    msg.data = "STOPPED";
   chatter_pub.publish(msg);
  }else if(STATE==BALL_APPROACH){
    msg.data =  "BALL_APPROACH";
   chatter_pub.publish(msg);
  }else if(STATE==BALL_RECOLECT){
   msg.data = "BALL_RECOLECT";
   chatter_pub.publish(msg);
  }else if(STATE==EXPLORING){
      msg.data =  "EXPLORING";
     chatter_pub.publish(msg);
  }else if(STATE==LOST){
      msg.data =  "LOST";
     chatter_pub.publish(msg);
  }else if(STATE==HOME_RETURN){
        msg.data =  "HOME_RETURN";
       chatter_pub.publish(msg);
  }

  CHANGE_STATE=FALSE;
}


    //ROS_INFO("%s", msg.data.c_str());

    ros::spinOnce();

    loop_rate.sleep();

  }
}
