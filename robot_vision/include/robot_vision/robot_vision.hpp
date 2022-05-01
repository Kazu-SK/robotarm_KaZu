
#include <chrono>
#include <memory>

#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_service/srv/coordinate_conversion.hpp"
//#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.hpp>

#include "cv_bridge/cv_bridge.h"
//#include "sensor_msgs/msg/camera_info.hpp"



using namespace std::placeholders;


class RobotVision : public rclcpp::Node{

private:

	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

	std::shared_ptr<rclcpp::Node> node;
	rclcpp::Client<vision_service::srv::CoordinateConversion>::SharedPtr client;

	cv::Mat split_image[3];  
	cv::Mat red_image[2];  //index 0:THRESH_BINARY  1:THRESH_BINARY_INV
	cv::Mat blue_image[2];
	cv::Mat green_image[2];

	cv::Mat redblock_image;


public:

	RobotVision() : Node("RobotVision"){

		node = rclcpp::Node::make_shared("coordinate_conversion_client");
		client = node->create_client<vision_service::srv::CoordinateConversion>("coordinate_conversion");

		image_sub = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&RobotVision::CallbackImage, this, _1));

	}

	void CallbackImage(const sensor_msgs::msg::Image::SharedPtr msg);


};

