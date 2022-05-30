
#include <chrono>
#include <memory>

#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "kinematics_service/srv/inv_kinematics.hpp"
#include "mg996r_messages/msg/mg996r_msg.hpp"
#include "vision_interfaces/msg/world_coordinate.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;


typedef struct MG996R_SERVO{

	int channel_num;
	double value_speed;
	double position;
	double offset;

	double diff_position; 
	double log_position;

}MG996R_SERVO;


enum Servo_id{

	SERVO_ID_0,
	SERVO_ID_1,
	SERVO_ID_2,
	SERVO_ID_3,
	SERVO_ID_4,
	SERVO_ID_5,
	SERVO_NUM
};



enum Action{

	ROBOT_INIT_POSITION,
	SET,
	CLOSER,
	LIFT,	
	MOVE,
	PUT,
	HAND_OPEN,
	HAND_CLOSE,
	NONE

};


class TestKinematics : public rclcpp::Node{


private:
	MG996R_SERVO robot_joint[6];
	int servo_num;

	std::shared_ptr<rclcpp::Node> node;
	rclcpp::Client<kinematics_service::srv::InvKinematics>::SharedPtr client;
	rclcpp::TimerBase::SharedPtr timer_;
	//kinematics_service::srv::InvKinematics::Request request;
	//std::shared_ptr<kinematics_service::srv::InvKinematics::Request> request;
	//
	rclcpp::Publisher<mg996r_messages::msg::Mg996rMsg>::SharedPtr  test_kinematics_pub;
	rclcpp::TimerBase::SharedPtr mg996r_pub_timer_;
	mg996r_messages::msg::Mg996rMsg mg996r_message;

	rclcpp::Subscription<vision_interfaces::msg::WorldCoordinate>::SharedPtr world_coordinate_subscription_;
	//rclcpp::TimerBase::SharedPtr world_coordinate_sub_timer_;

	//vision_interfaces::msg::WorldCoordinate world_coordinate_message;


	double target_x;
	double target_y;
	double target_z;
	double target_pitch;
	double target_yaw;

	double world_x;
	double world_y;
	double world_z;


	enum Action ac;

public:
	TestKinematics()
	: Node("test_kinematics")
	{
		node = rclcpp::Node::make_shared("inv_kinematics_client");

		RCLCPP_INFO(this->get_logger(), "test_kinematics");
		client = node->create_client<kinematics_service::srv::InvKinematics>("inverse_kinematics");
		TestKinematicsInitialize();
		//request = std::make_shared<kinematics_service::srv::InvKinematics::Request>();
		timer_ = create_wall_timer(5s, std::bind(&TestKinematics::ActionMain, this));

		test_kinematics_pub = this->create_publisher<mg996r_messages::msg::Mg996rMsg>("mg996r_topic", 100);
		mg996r_pub_timer_ = this->create_wall_timer(120ms, std::bind(&TestKinematics::Mg996rPub, this));
		mg996r_message = mg996r_messages::msg::Mg996rMsg();

		
		world_coordinate_subscription_ = this->create_subscription<vision_interfaces::msg::WorldCoordinate>("world_coordinate_topic", 100, std::bind(&TestKinematics::WorldCoordinateSub, this, _1));
		
		//world_coordinate_sub_timer_ = this->create_wall_timer(120ms, std::bind(&TestKinematics::WorldCoordinateSub, this, _1));

		

		//world_coordinate_message = vision_interfaces::msg::WorldCoordinate();

		ac = ROBOT_INIT_POSITION;

	}

	void TestKinematicsInitialize();

	void Request();

	void CalcSpeed();

	void InitPosition();
	void HandClose();
	void HandOpen();

	void ActionMain();

	void WorldCoordinateSub(const vision_interfaces::msg::WorldCoordinate::SharedPtr msg);
	void Mg996rPub();

};
