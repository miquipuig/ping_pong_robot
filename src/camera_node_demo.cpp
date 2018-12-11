#include "camera_node.h"

const int GAUSSIAN_BLUR_SIZE = 9 ;
const double GAUSSIAN_BLUR_SIGMA = 2;
const double CANNY_EDGE_TH = 120; //subir menos bolas #Para1
const double HOUGH_ACCUM_RESOLUTION = 2;
const double MIN_CIRCLE_DIST = 40;
const double HOUGH_ACCUM_TH = 60; //subir para menos bolas #Para2
const int MIN_RADIUS = 30; //minimo radio de pelota.
const int MAX_RADIUS = 50; //maximo radio de las pelotas.
const double xcenter=640/2-60; //resolución camera en x
const double ycenter=480/2; //resolución camera en y
const double  newycenter=ycenter+94; //punto en y donde se encuentra la X
const double cross= 15; //anchura de cruz central
const double linewide= 4;
const int MAX_BALLS=6; //Numero de pelotas que se analiza por cercania. Como mas grande mas posibles candidatas falsas.
const int ZEROS_TIME=5; //Minimo de zeros seguidos para enviar dirección nula.
const int BALLS_TIME=3; //Minimo de veces que se he de ver pelota para enviar dirección
const int ZEROS_RESET_TIME=2; //Zeros seguidos para los cuales se resetea BALLS_TIME. Como mas grande mas cuesta encontrar candidato
const int MAX_DECTIONS_TO_AVOID=6; //Número de detecciones máximo de bolas para suponer sensor saturado.
int ballscount=0;
int zeroscount=0;

RosImgProcessorNode::RosImgProcessorNode() :
    nh_(ros::this_node::getName()),
    img_tp_(nh_)
{
  //loop rate [hz], Could be set from a yaml file
	rate_=10;

	//sets publishers
	image_pub_ = img_tp_.advertise("image_out", 100);
  ray_direction_circle_pub = nh_.advertise<geometry_msgs::Vector3>("raydirector", 1);
  nextBall = nh_.advertise<geometry_msgs::Vector3>("direction", 1);
  ray_direction_ = (cv::Mat_<double>(3,1) << 0, 0, 0) ;

	//sets subscribers
	image_subs_ = img_tp_.subscribe("image_in", 1, &RosImgProcessorNode::imageCallback, this);
	camera_info_subs_ = nh_.subscribe("camera_info_in", 100, &RosImgProcessorNode::cameraInfoCallback, this);

}

RosImgProcessorNode::~RosImgProcessorNode()
{

}

double hypotenuse( double x, double y, double cx, double cy ){


      return pow( cx-x, 2 ) + pow( cy-y, 2 );
}

void RosImgProcessorNode::process()
{
    cv::Rect_<int> box;
    cv::Point center;
    cv::Mat gray_image;
    int radius;
    std::vector<cv::Vec3f> circles;
    cv::Mat K = matrixK_;
    cv::Mat u=(cv::Mat_<double>(3,1)<< 0,0,0);



    int balls_size=0;
    cv::Point centerp = cv::Point(xcenter, ycenter);
    cv::Point newcenter = cv::Point(xcenter, newycenter);
    cv::Point vx1 = cv::Point(xcenter-cross, newycenter);
    cv::Point vx2 = cv::Point(xcenter+cross, newycenter);
    cv::Point vy1 = cv::Point(xcenter, newycenter-cross);
    cv::Point vy2 = cv::Point(xcenter, newycenter+cross);
    //check if new image is there
    if ( cv_img_ptr_in_ != nullptr )
    {
        //copy the input image to the out one
        cv_img_out_.image = cv_img_ptr_in_->image;

		//find the ball
		//TODO
    circles.clear();
    cv::cvtColor(cv_img_ptr_in_->image, gray_image, CV_BGR2GRAY);
    cv::GaussianBlur( gray_image, gray_image, cv::Size(GAUSSIAN_BLUR_SIZE, GAUSSIAN_BLUR_SIZE), GAUSSIAN_BLUR_SIGMA );
    cv::HoughCircles( gray_image, circles, CV_HOUGH_GRADIENT, HOUGH_ACCUM_RESOLUTION, MIN_CIRCLE_DIST, CANNY_EDGE_TH, HOUGH_ACCUM_TH, MIN_RADIUS, MAX_RADIUS );

		//find the direction vector
		//TODO
     cv::Mat Kinv= K.inv();
     //std::cout << Kinv << std::endl;
     //exemple de u
    if(circles.size()>MAX_BALLS){
        balls_size=MAX_BALLS;

     }
     else{
       balls_size=circles.size();
     }
     //ROS_INFO("1-balls_Size: %d",balls_size);
    //balls_size=circles.size();


    geometry_msgs::Vector3 selectionBalls[balls_size];

    double miniumDistance=1000;
    int closeBall=0;


    for(unsigned int ii = 0; ii < balls_size; ii++ )
    {

        if ( circles[ii][0] != -1 )
        {
                //std::cout << "Circulo: " << circles[ii][0] <<";"<< circles[ii][1]<<";"<< circles[ii][2]<<std::endl;
                //u=(cv::Mat_<double>(3,1)<< circles[ii][0] -xcenter, circles[ii][1] - ycenter,1);
                u=(cv::Mat_<double>(3,1)<< circles[ii][0] , circles[ii][1] ,1);


                selectionBalls[ii].x=circles[ii][0];
                selectionBalls[ii].y=circles[ii][1];
                selectionBalls[ii].z=circles[ii][2];
                double hip=hypotenuse(circles[ii][0],circles[ii][1],newcenter.x, newcenter.y);
                if(hip<miniumDistance){
                  miniumDistance=hip;
                  closeBall=ii;
                }

                center = cv::Point(cvRound(circles[ii][0]), cvRound(circles[ii][1]));
                radius = cvRound(circles[ii][2]);
                cv::circle(cv_img_out_.image, center, 5, cv::Scalar(0,0,255), -1, linewide, 0 );// circle center in green
                cv::circle(cv_img_out_.image, center, radius, cv::Scalar(0,0,255), 3, linewide, 0 );// circle perimeter in red
                //vector Ray director
                cv::line(cv_img_out_.image,newcenter,center,cv::Scalar(0,0,255), linewide); //linea

                /*geometry_msgs::Vector3 direction;
                direction.x = ray_direction_.at<double>(0, 0);
                direction.y = ray_direction_.at<double>(1, 0);
                direction.z = ray_direction_.at<double>(2, 0);
                ray_direction_circle_pub.publish(direction);*/

        }
    }

      if (balls_size>0&&balls_size<MAX_DECTIONS_TO_AVOID){
        ballscount+=1;

      //ROS_INFO("Imprimir ball");
          if(ballscount>=BALLS_TIME){

              center = cv::Point(cvRound(selectionBalls[closeBall].x), cvRound(selectionBalls[closeBall].y));
              radius = cvRound(selectionBalls[closeBall].z);
              cv::circle(cv_img_out_.image, center, 5, cv::Scalar(255,0,0), -1, linewide, 0 );// circle center in green
              cv::circle(cv_img_out_.image, center, radius, cv::Scalar(255,0,0), 3, linewide, 0 );// circle perimeter in red
              //vector Ray director
              cv::line(cv_img_out_.image,newcenter,center,cv::Scalar(255,0,0), linewide); //linea

              geometry_msgs::Vector3 direction;
              direction.x = selectionBalls[closeBall].x-newcenter.x;
              direction.y = newcenter.y-selectionBalls[closeBall].y;
              direction.z =balls_size;
              nextBall.publish(direction);
              zeroscount=0;
          }
      }else{
        //ROS_INFO("1 -  No hay volas - contador: %d",zeroscount);
        zeroscount+=1;
        if(zeroscount>=ZEROS_RESET_TIME){
          ballscount=0;
        }
        //ROS_INFO("2 -  No hay volas - contador: %d",zeroscount);

        if(zeroscount>=ZEROS_TIME){
            //ROS_INFO("2 -  No hay volas - contador: %d - Imprimo topic",zeroscount);
          zeroscount=0;

          geometry_msgs::Vector3 direction;
          direction.x = 0;
          direction.y = 0;
          direction.z =0 ;
          nextBall.publish(direction);
        }

      }
      //Mirila central
      cv::line(cv_img_out_.image,vx1,vx2,cv::Scalar(255,0,0), linewide);
      cv::line(cv_img_out_.image,vy1,vy2,cv::Scalar(255,0,0), linewide);
    }

    //reset input image
    cv_img_ptr_in_ = nullptr;
}

void RosImgProcessorNode::publish()
{
    //image_raw topic
	if(cv_img_out_.image.data)
	{
	    cv_img_out_.header.seq ++;
	    cv_img_out_.header.stamp = ros::Time::now();
	    cv_img_out_.header.frame_id = "camera";
	    cv_img_out_.encoding = img_encoding_;
	    image_pub_.publish(cv_img_out_.toImageMsg());

	}
}

double RosImgProcessorNode::getRate() const
{
    return rate_;
}

void RosImgProcessorNode::imageCallback(const sensor_msgs::ImageConstPtr& _msg)
{
    try
    {
        img_encoding_ = _msg->encoding;//get image encodings
        cv_img_ptr_in_ = cv_bridge::toCvCopy(_msg, _msg->encoding);//get image
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("RosImgProcessorNode::image_callback(): cv_bridge exception: %s", e.what());
        return;
    }
}

void RosImgProcessorNode::cameraInfoCallback(const sensor_msgs::CameraInfo & _msg)
{
	matrixP_ = (cv::Mat_<double>(3,3) << _msg.P[0],_msg.P[1],_msg.P[2],
                                        _msg.P[3],_msg.P[4],_msg.P[5],
                                        _msg.P[6],_msg.P[7],_msg.P[8]);

  matrixK_ = (cv::Mat_<double>(3,3) << _msg.K[0],_msg.K[1],_msg.K[2],
                                      _msg.K[3],_msg.K[4],_msg.K[5],
                                      _msg.K[6],_msg.K[7],_msg.K[8]);

}
