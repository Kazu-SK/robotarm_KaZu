
#include "robot_data/robot_data.hpp"


int main(int argc, char **argv){

	rclcpp::Service<kinematics_service::srv::InvKinematics>::SharedPtr kinema_service;
	rclcpp::Subscription<vision_interfaces::msg::ImageCoordinate>::SharedPtr image_coordinate_sub;
	rclcpp::Publisher<vision_interfaces::msg::WorldCoordinate>::SharedPtr world_coordinate_pub;
	std::shared_ptr<rclcpp::Node> robot_data_server_node;
	std::shared_ptr<rclcpp::Node> robot_data_node;

	vision_interfaces::msg::WorldCoordinate world_coordinate_msg;
	RobotData robot_data;
	double world_coordinate[3] = {0.0, 0.0, 0.0};


	rclcpp::init(argc, argv);



	robot_data_server_node = rclcpp::Node::make_shared("robot_data_server");
	robot_data_node = rclcpp::Node::make_shared("robot_data_node");

	kinema_service = robot_data_server_node->create_service<kinematics_service::srv::InvKinematics>("inverse_kinematics", std::bind(&RobotData::InvKinemaService, &robot_data, _1, _2));
	image_coordinate_sub = robot_data_node->create_subscription<vision_interfaces::msg::ImageCoordinate>("image_coordinate_topic", 100, std::bind(&RobotData::CoordinateConversion, &robot_data, _1));
	world_coordinate_pub = robot_data_node->create_publisher<vision_interfaces::msg::WorldCoordinate>("world_coordinate_topic", 100);

	while(rclcpp::ok()){

		rclcpp::spin_some(robot_data_server_node);
		rclcpp::spin_some(robot_data_node);

		robot_data.getWorldCoordinate(world_coordinate);

		world_coordinate_msg.world_x = world_coordinate[0];
		world_coordinate_msg.world_y = world_coordinate[1];
		world_coordinate_msg.world_z = world_coordinate[2];

        	world_coordinate_pub->publish(world_coordinate_msg);	
	}

	rclcpp::shutdown();

	return 0;
}
