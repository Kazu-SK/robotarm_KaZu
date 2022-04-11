
#include "robot_vision/robot_vision.hpp"



void RobotVision::CallbackImage(const sensor_msgs::msg::Image::SharedPtr msg){

	
	
	cv_bridge::CvImagePtr cv_ptr;

	//RCLCPP_INFO(this->get_logger(),"encoding = %c",msg->encoding );
	//RCLCPP_INFO(this->get_logger(),"encoding");

	try{
		//cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr16");
	}
	catch(cv_bridge::Exception& e)
	{
		RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
		return;
	}
	


	cv::imshow("window", cv_ptr->image);

	cv::waitKey(3);


}
