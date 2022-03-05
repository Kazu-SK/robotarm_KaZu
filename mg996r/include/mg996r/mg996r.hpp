
#include <chrono>
#include <memory>

#include <wiringSerial.h>

#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"

#include "mg996r_messages/msg/mg996r_msg.hpp"

using std::placeholders::_1;

typedef struct WITMOTION_COMMAND{

	unsigned char speed_init_cmd; 
	unsigned char speed_cmd; 
	unsigned char speed_ch; 
	unsigned char speed_DataL; 
	unsigned char speed_DataH; 

	unsigned char position_init_cmd;
	unsigned char position_cmd;
	unsigned char position_ch;
	unsigned char position_DataL;
	unsigned char position_DataH;


}Witmotion_command;


class Mg996rClass : public rclcpp::Node{

private:

	unsigned char write_data[10];


	int fd;
	float SLOPE_A;
	float INTERCEPT_B;

	rclcpp::Subscription<mg996r_messages::msg::Mg996rMsg>::SharedPtr mg996r_sub;

public:
	Mg996rClass() : Node("Mg996rClass"){


		Mg996rInitialize();

		mg996r_sub = this->create_subscription<mg996r_messages::msg::Mg996rMsg>("mg996r_topic", 100, std::bind(&Mg996rClass::Mg996rOperation, this, _1));

	
	}

	~Mg996rClass(){

		serialClose(fd);
	}

	//Mg996r
	static const int LOW_PULSE;
	static const int HIGH_PULSE;
	static const float LOW_POSITION;
  	static const float HIGH_POSITION;


	//WITMOTION module
	static const unsigned char INIT_CMD;
	static const unsigned char SPEED_CMD;
	static const unsigned char POSITION_CMD;

	
	void Mg996rInitialize();
	void Mg996rOperation(const mg996r_messages::msg::Mg996rMsg::SharedPtr msg);

	//int EmergencyStop();

	unsigned short TwopointInterpolation(float position){return (unsigned short)(SLOPE_A * position + INTERCEPT_B); }


	int SerialWrite(unsigned char write_data[], int len);

};

