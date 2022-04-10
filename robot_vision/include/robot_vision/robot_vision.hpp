
#include <chrono>
#include <memory>

#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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


public:

	void CallbackImage(const sensor_msgs::msg::Image::SharedPtr msg);


	RobotVision() : Node("RobotVision"){
		image_sub = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&RobotVision::CallbackImage, this, _1));

	}

};

