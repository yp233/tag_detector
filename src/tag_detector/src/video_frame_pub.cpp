/////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Answer for Bito Company Recuitment 
//
// Author: yp
//
// Node: video_frame_pub 
//
// Finished tasks on this node:
//             1、 publish 20s video on topic "/image_raw"
//             2、 add a Gaussian blur on the input image (default size = 0,0)
//             3、 create a ROS service server "/set_blur_window_size"
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include<stdio.h> 
#include<ros/ros.h> 
#include<cv_bridge/cv_bridge.h> 
#include<sensor_msgs/image_encodings.h> 
#include<image_transport/image_transport.h> 
#include<opencv2/opencv.hpp> 
#include<opencv2/highgui/highgui.hpp> 
#include<opencv2/imgproc/imgproc.hpp> 
#include<tag_detector/ServiceMsg.h>

using namespace cv; 
using namespace std;

#define RED                  "\e[0;31m"
#define GREEN                "\e[0;32m"

int x_size=0;
int y_size=0;

bool servicecallback(tag_detector::ServiceMsgRequest& request,tag_detector::ServiceMsgResponse& response)
{
	ROS_INFO("callback activated");
	
	if (request.size1>0 && request.size1%2==1 && request.size2>0 && request.size2%2==1)
	{
		response.isSetOK=true;
		x_size=request.size1;
		y_size=request.size2;
	}
	else
	{	
		ROS_INFO("please set value correctly");
		response.isSetOK=false;
	}
	return true;
}

int main(int argc, char** argv) 
{ 
	ros::init(argc, argv, "video_frame_pub"); 
	ros::NodeHandle n;
	ros::Time time = ros::Time::now(); 
	ros::Rate loop_rate(5); 

	image_transport::ImageTransport it(n); 
	image_transport::Publisher pub = it.advertise("image_raw", 1); //publish images on topic /image_raw

	sensor_msgs::ImagePtr msg; 
	cv::VideoCapture video; 
	video.open("src/AprilTags.mp4"); 
	if( !video.isOpened() ) 
	{ 
        ROS_INFO("Read Video failed!\n"); 
		return 0; 
    }

	Mat frame,gblur; 
	int count = 0; 
	while(1) { 
		video >> frame; 
		if( frame.empty() ) 
			break; 
		count++; 
        ros::ServiceServer service = n.advertiseService("set_blur_window_size", servicecallback);//sevice set_blur_window_size
        printf(RED"blur_window_size:"GREEN"%d"","GREEN"%d\n",x_size,y_size);
        
		if(x_size)
		{	
			GaussianBlur(frame,gblur,Size(x_size,y_size),0,0);
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", gblur).toImageMsg();
		} 
		else
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg(); 	
		
		pub.publish(msg); 
		ROS_INFO( "read the %dth frame successfully!", count ); 
		loop_rate.sleep(); 
		ros::spinOnce(); 
	} 
	video.release(); 
	return 0;
 }

