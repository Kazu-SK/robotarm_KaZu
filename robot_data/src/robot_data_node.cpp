
#include "robot_data/robot_data.hpp"


int main(int argc, char **argv){

	
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotData>());

	rclcpp::shutdown();
	

	

	return 0;
}
