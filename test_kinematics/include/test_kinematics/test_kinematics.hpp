
#include <chrono>
#include <memory>

#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "kinematics_service/srv/inv_kinematics.hpp"
#include "mg996r_messages/msg/mg996r_msg.hpp"

using namespace std::chrono_literals;




typedef struct MG996R_SERVO{

	int channel_num;
	unsigned short value_speed;
	double position;
	double offset;

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
	HAND_CLOSE

};


class TestKinematics : public rclcpp::Node{


private:
	MG996R_SERVO robot_joint[6];
	int servo_num;

	std::shared_ptr<rclcpp::Node> node;
	rclcpp::Client<kinematics_service::srv::InvKinematics>::SharedPtr client;
	//kinematics_service::srv::InvKinematics::Request request;
	//std::shared_ptr<kinematics_service::srv::InvKinematics::Request> request;
	//
	rclcpp::Publisher<mg996r_messages::msg::Mg996rMsg>::SharedPtr  test_operation_pub;
	rclcpp::TimerBase::SharedPtr timer_;
	mg996r_messages::msg::Mg996rMsg mg996r_message;

	rclcpp::TimerBase::SharedPtr mg996r_pub_timer_;

	double target_x;
	double target_y;
	double target_z;
	double target_pitch;
	double target_yaw;

	enum Action ac;

public:
	TestKinematics()
	: Node("test_kinematics")
	{
		TestKinematicsInitialize();
		node = rclcpp::Node::make_shared("inv_kinematics_client");
		client = node->create_client<kinematics_service::srv::InvKinematics>("inverse_kinematics");
		//request = std::make_shared<kinematics_service::srv::InvKinematics::Request>();
		timer_ = create_wall_timer(1s, std::bind(&TestKinematics::ActionMain, this));

		test_operation_pub = this->create_publisher<mg996r_messages::msg::Mg996rMsg>("mg996r_topic", 100);
		mg996r_pub_timer_ = this->create_wall_timer(120ms, std::bind(&TestKinematics::Publish, this));
		mg996r_message = mg996r_messages::msg::Mg996rMsg();

		ac = ROBOT_INIT_POSITION;
		/*

		while(!client->wait_for_service(1s)){

			if(!rclcpp::ok()){
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
		}
		*/

	}

	void TestKinematicsInitialize();

	void Request();
	void ActionMain();

	void Publish();
};
