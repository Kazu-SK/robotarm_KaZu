
#include "robot_data/robot_data.hpp"


int main(int argc, char **argv){

	
	rclcpp::Service<kinematics_service::srv::InvKinematics>::SharedPtr service;
	std::shared_ptr<rclcpp::Node> node;


	rclcpp::init(argc, argv);

	RobotData robot_data;


	node = rclcpp::Node::make_shared("robot_data_server");

	service = node->create_service<kinematics_service::srv::InvKinematics>("inverse_kinematics", std::bind(&RobotData::InvKinemaService, &robot_data, _1, _2));

	rclcpp::spin(node);

	rclcpp::shutdown();
	

	

	return 0;
}
