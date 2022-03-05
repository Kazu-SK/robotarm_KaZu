

#include "mg996r/mg996r.hpp"


int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Mg996rClass>());
	rclcpp::shutdown();

	return 0;

}
