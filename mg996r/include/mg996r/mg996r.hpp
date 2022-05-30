
#include <chrono>
#include <memory>

#include <wiringSerial.h>

#include <unistd.h>
#include <termios.h>

#include "rclcpp/rclcpp.hpp"

#include "mg996r_messages/msg/mg996r_msg.hpp"

using namespace std::placeholders;

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


}WITMOTION_COMMAND;


class Mg996rClass : public rclcpp::Node{

private:

	unsigned char write_data[10];


	int fd;
	double POSITION_SLOPE_A;
	double POSITION_INTERCEPT_B;
	double SPEED_SLOPE_A;
	double SPEED_INTERCEPT_B;

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
	static const double LOW_POSITION;
  	static const double HIGH_POSITION;

	static const int LOW_VALUE;
	static const int HIGH_VALUE;
	static const double LOW_SPEED;
  	static const double HIGH_SPEED;


	//WITMOTION module
	static const unsigned char INIT_CMD;
	static const unsigned char SPEED_CMD;
	static const unsigned char POSITION_CMD;

	
	void Mg996rInitialize();
	void Mg996rOperation(const mg996r_messages::msg::Mg996rMsg::SharedPtr msg);

	//int EmergencyStop();

	unsigned short TwopointInterpolation(double x, double a, double b);


	int SerialWrite(unsigned char write_data[], int len);

};

