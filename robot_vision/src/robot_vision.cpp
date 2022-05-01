
#include "robot_vision/robot_vision.hpp"




void RobotVision::CallbackImage(const sensor_msgs::msg::Image::SharedPtr msg){

	cv_bridge::CvImagePtr cv_ptr;
	auto request = std::make_shared<vision_service::srv::CoordinateConversion::Request>();
	//RCLCPP_INFO(this->get_logger(),"encoding = %c",msg->encoding );
	//RCLCPP_INFO(this->get_logger(),"encoding");

	try{
		//cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
		cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
	}
	catch(cv_bridge::Exception& e)
	{
		RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
		return;
	}

	//cv::Mat gray_img;
	cv::Mat copy_img;
	cv::Mat filter_img;
	cv::Mat mask_img;
	int filter_size = 9;
	cv::Mat thresh_img;

	cv_ptr->image.copyTo(copy_img);
	//
	//
	
	cv::split(copy_img, split_image);

	cv::threshold(split_image[2], red_image[0], 130, 255, CV_THRESH_BINARY);
	cv::threshold(split_image[2], red_image[1], 255, 255, CV_THRESH_BINARY_INV);
	cv::threshold(split_image[1], green_image[0], 0, 255, CV_THRESH_BINARY);
	cv::threshold(split_image[1], green_image[1], 50, 255, CV_THRESH_BINARY_INV);
	cv::threshold(split_image[0], blue_image[0], 0, 255, CV_THRESH_BINARY);
	cv::threshold(split_image[0], blue_image[1], 50, 255, CV_THRESH_BINARY_INV);

	redblock_image = (red_image[0] & red_image[1]) & (blue_image[0] & blue_image[1]) & (green_image[0] & green_image[1]);
	//cv::inRange(cv_ptr->image, cv::Scalar(0,0, 130), cv::Scalar(50, 50, 255), mask_img);

	cv::GaussianBlur(redblock_image, filter_img, cv::Size(filter_size, filter_size), 5); 



	int contours_count = 0;
	int detect_x;
	int detect_y;
	std::vector<std::vector<cv::Point>> object_contours;

	findContours(filter_img, object_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	for(long unsigned int contours_num = 0 ; contours_num < object_contours.size(); contours_num++){
		
		contours_count = 0;	

		contours_count += object_contours.at(contours_num).size();

		if(contours_count > 230 && contours_count < 280){

			for(int detect_contours = 0 ; detect_contours < contours_count ; detect_contours++){
				detect_x += object_contours.at(contours_num).at(detect_contours).x;
				detect_y += object_contours.at(contours_num).at(detect_contours).y;
			}

			detect_x /= contours_count;
			detect_y /= contours_count;

			cv::circle(copy_img, cv::Point(detect_x, detect_y), 10, cv::Scalar(0,255,0),2,4);
			RCLCPP_INFO(this->get_logger(), "detect_x = %d", detect_x);
			RCLCPP_INFO(this->get_logger(), "detect_y = %d", detect_y);
		}
	}

	
	
	/*
	while(!client->wait_for_service(1s)){

		if(!rclcpp::ok()){
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			break;
		}
		RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
	}
	*/

	request->camera_u = detect_x;
	request->camera_v = detect_y;
	

	auto result = client->async_send_request(request); 

	if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS){
		
		RCLCPP_INFO(this->get_logger(), "***********************");
		RCLCPP_INFO(this->get_logger(), "world_x = %f", result.get()->world_x);
		RCLCPP_INFO(this->get_logger(), "world_y = %f", result.get()->world_y);


	}
	else{

		RCLCPP_ERROR(this->get_logger(), "Failed");
	}

	cv::imshow("window", copy_img);

	cv::waitKey(3);

}
