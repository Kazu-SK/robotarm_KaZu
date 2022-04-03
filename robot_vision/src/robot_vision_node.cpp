
#include "robot_vision/robot_vision.hpp"


int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotVision>());

	rclcpp::shutdown();

	return 0;
}
