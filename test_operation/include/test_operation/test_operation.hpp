
#include <chrono>
#include <memory>
#include <functional>

#include <wiringSerial.h>

#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"
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

class TestOperation : public rclcpp::Node{

private:
	MG996R_SERVO robot_joint[6];

	int servo_num;
	
	rclcpp::Publisher<mg996r_messages::msg::Mg996rMsg>::SharedPtr  test_operation_pub;
	rclcpp::TimerBase::SharedPtr timer_;
	mg996r_messages::msg::Mg996rMsg mg996r_message;

public:
	TestOperation()
	: Node("test_operation")
	{
		test_operation_pub = this->create_publisher<mg996r_messages::msg::Mg996rMsg>("mg996r_topic", 100);
		timer_ = this->create_wall_timer(120ms, std::bind(&TestOperation::Publish, this));
		mg996r_message = mg996r_messages::msg::Mg996rMsg();

		TestOperationInitialize();
	}

	void TestOperationInitialize();
	void Publish();

};

