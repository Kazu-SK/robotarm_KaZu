
#include "robot_data/robot_data.hpp"


int main(int argc, char **argv){

	//rclcpp::Service<kinematics_service::srv::InvKinematics>::SharedPtr kinema_service;
	//rclcpp::Service<vision_service::srv::CoordinateConversion>::SharedPtr convert_service;
	//std::shared_ptr<rclcpp::Node> robot_data_server_node;
	std::shared_ptr<rclcpp::Node> robot_data_node;

	rclcpp::init(argc, argv);


	//RobotData robot_data;

	//robot_data_server_node = rclcpp::Node::make_shared("robot_data_server");
	robot_data_node = std::make_shared<RobotData>();


	//kinema_service = robot_data_server_node->create_service<kinematics_service::srv::InvKinematics>("inverse_kinematics", std::bind(&RobotData::InvKinemaService, &robot_data, _1, _2));
	//convert_service = node->create_service<vision_service::srv::CoordinateConversion>("coordinate_conversion", std::bind(&RobotData::CoordinateConversionService, &robot_data, _1, _2));

	while(rclcpp::ok()){

		//rclcpp::spin_some(robot_data_server_node);
		rclcpp::spin_some(robot_data_node);

	}
	//rclcpp::spin(node);
	//rclcpp::spin(std::make_shared<RobotData>());
	rclcpp::shutdown();

	

	return 0;
}
