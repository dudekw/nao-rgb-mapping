
#include "ros/ros.h"
#include "rapp_ros_naoqi_wrappings/MoveVel.h"
#include "rapp_ros_naoqi_wrappings/GetImage.h"
#include "rapp_ros_naoqi_wrappings/MoveJoint.h"
#include "rapp_ros_naoqi_wrappings/TakePredefinedPosture.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <sstream>
#include <stdlib.h>
#include <ros/console.h>
#include <boost/filesystem.hpp>
void move(float x, float vel, ros::ServiceClient &client){
	float interval = x/vel;
	rapp_ros_naoqi_wrappings::MoveVel srv;
  	srv.request.velocity_x = vel;
  	srv.request.velocity_y = 0;
  	srv.request.velocity_theta = 0;
	
	if (client.call(srv))
	{
		ROS_INFO_STREAM("Move status: "<< srv.response.status);
	}
	else
	{
		ROS_ERROR_STREAM("Failed to call service rapp_moveVel -> setting velocity aborted");
	}

	ros::Duration(interval).sleep();
  	srv.request.velocity_x = 0;
  	srv.request.velocity_y = 0;
  	srv.request.velocity_theta = 0;

	if (client.call(srv))
	{
		ROS_INFO_STREAM("Stop status: "<< srv.response.status);
	}
	else
	{
		ROS_ERROR_STREAM("Failed to call service rapp_moveVel -> stopping robot aborted");
	}
}

sensor_msgs::Image capture(ros::ServiceClient &client){

	rapp_ros_naoqi_wrappings::GetImage srv;
	//srv.request.camera_id = 0;
	srv.request.resolution = 3;
	
	if (client.call(srv))
	{
		ROS_INFO_STREAM("Capture successfull");
		return srv.response.frame;
	}
	else
	{
		ROS_ERROR_STREAM("Failed to call service rapp_captureImage");
	  	return srv.response.frame;
	}
}

void moveCamera(float angle, ros::ServiceClient &client){

	rapp_ros_naoqi_wrappings::MoveJoint srv;
	std::vector<std::string> names;
	names.clear();
	names.push_back("HeadYaw");
	std::vector<float> angles;
	angles.clear();
	angles.push_back(angle);
	srv.request.joint_name = names;
	srv.request.joint_angle = angles;
	srv.request.speeds = 0.7;
	
	if (client.call(srv))
	{
		ROS_INFO_STREAM("Move joint responded with status: "<< srv.response.status);
	}
	else
	{
		ROS_ERROR_STREAM("Failed to call service rapp_moveJoint");
	}
}

void saveImage(std::string path, sensor_msgs::Image ros_image){
	cv::Mat cv_image;
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(ros_image, sensor_msgs::image_encodings::BGR8);
	cv_image = cv_ptr->image;
	cv::imwrite( path, cv_image );
}

void handlePoint(int iterate_init, std::string pointId, ros::ServiceClient &client_capture, ros::ServiceClient &client_moveJoint,std::string pic_folder_path){
	sensor_msgs::Image ros_image;
	float angle = 0;
	std::string img_path;
	img_path = pic_folder_path + "/" + pointId;
	int i = 0;
	std::string orient_id;
	std::ostringstream ss;

	for(i; i <  6; i++){
		angle = -(-M_PI/2 + i * 30 * M_PI/180);
		std::cout<< "angle :"<<angle<<std::endl;
		moveCamera( angle, client_moveJoint);
		ros_image = capture(client_capture);
		if (iterate_init == 0){
			if (i > 3){
				ss.str("");
				ss << 15 - i;
				orient_id = ss.str();
			}
			else{
				ss.str("");
				ss << 3 - i;
				orient_id = "0"+ss.str();
				std::cout<<img_path + "_" + orient_id + ".jpg"<<std::endl;
			}
			saveImage(img_path + "_" + orient_id + ".jpg",ros_image );
		}
		else {
			ss.str("0");
			ss << 9 - i;
			orient_id = ss.str(); 
			saveImage(img_path + "_0" + orient_id + ".jpg",ros_image);
		}
	}

}
void handlePosture(std::string posture, ros::ServiceClient &client_posture){

	rapp_ros_naoqi_wrappings::TakePredefinedPosture srv;
	srv.request.pose = posture;
	srv.request.speed = 0.6;

	if (client_posture.call(srv))
	{
		ROS_INFO_STREAM("Take posture responded with status: "<< srv.response.status);
	}
	else
	{
		ROS_ERROR_STREAM("Failed to call service rapp_takePredefinedPosture");
	}
}
int main(int argc, char** argv){
ros::init(argc, argv, "my_node_name");
ros::NodeHandle nh;
std::cout << "Usage: <<direction>> <<col_ID>>, <<row_ID>>, direction: 0-> to E || 9-> to W"<< std::endl;
ros::ServiceClient client_moveVel = nh.serviceClient<rapp_ros_naoqi_wrappings::MoveVel>("rapp_moveVel");
ros::ServiceClient client_posture = nh.serviceClient<rapp_ros_naoqi_wrappings::TakePredefinedPosture>("rapp_takePredefinedPosture");
ros::ServiceClient client_moveJoint = nh.serviceClient<rapp_ros_naoqi_wrappings::MoveJoint>("rapp_moveJoint");
ros::ServiceClient client_capture = nh.serviceClient<rapp_ros_naoqi_wrappings::GetImage>("rapp_capture_image");
std::string pic_folder_path = "./mapped_pic"; 


const char *dir_path = pic_folder_path.c_str();
	boost::filesystem::path dir(dir_path);
	if(boost::filesystem::create_directory(dir)) {
		std::cout << "Success" << "\n";
	}
int direction = atoi(argv[1]);
std::string pointID = "";
int cols = 9;
std::ostringstream ss;
ss.str("");
int col = atoi(argv[2]);
ss<<col;
int row = atoi(argv[3]);
ss<<row;

pointID = ss.str();
handlePosture("Stand", client_posture);
handlePoint(direction, pointID, client_capture, client_moveJoint, pic_folder_path);
moveCamera( -M_PI/2 , client_moveJoint);


return 0;
}
