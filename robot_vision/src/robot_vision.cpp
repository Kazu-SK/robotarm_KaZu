
#include "robot_vision/robot_vision.hpp"



void RobotVision::CallbackImage(const sensor_msgs::msg::Image::SharedPtr msg){

	
	
	cv_bridge::CvImagePtr cv_ptr;


	cv_ptr = cv_bridge::toCvCopy(msg, "BGR8");
	

	
	RCLCPP_INFO(this->get_logger(), "height = %d", msg->height);


	/*
	try{

		cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	}
	catch (cv_bridge::Exception& e){
		
		
	}	
	*/

	//RCLCPP_INFO(this->get_logger(), "height = %d", msg->height);

}
