
#include "test_kinematics/test_kinematics.hpp"


int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TestKinematics>());


	rclcpp::shutdown();


	return 0;
}
