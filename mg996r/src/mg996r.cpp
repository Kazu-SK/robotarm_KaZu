
#include "mg996r/mg996r.hpp"


//Two-point interpolation (position) 
const int Mg996rClass::LOW_PULSE = 544;
const int Mg996rClass::HIGH_PULSE = 2400;
const double Mg996rClass::LOW_POSITION = 0.0;
const double Mg996rClass::HIGH_POSITION = 180.0;

//Two-point interpolation (speed) 
const int Mg996rClass::LOW_VALUE = 1;
const int Mg996rClass::HIGH_VALUE = 20;
const double Mg996rClass::LOW_SPEED = 9.0;
const double Mg996rClass::HIGH_SPEED = 180.0;

//WITMOTION module
const unsigned char Mg996rClass::INIT_CMD = 0xFF;
const unsigned char Mg996rClass::SPEED_CMD = 0x01;
const unsigned char Mg996rClass::POSITION_CMD = 0x02;


void Mg996rClass::Mg996rInitialize(){

	POSITION_SLOPE_A = (HIGH_PULSE - LOW_PULSE) / (HIGH_POSITION - LOW_POSITION);	
	POSITION_INTERCEPT_B = HIGH_PULSE - POSITION_SLOPE_A * HIGH_POSITION;

	SPEED_SLOPE_A = (HIGH_VALUE - LOW_VALUE) / (HIGH_SPEED - LOW_SPEED);
	SPEED_INTERCEPT_B = HIGH_VALUE - SPEED_SLOPE_A * HIGH_SPEED;

	//Serial
	struct termios ttyparam;

	fd = serialOpen("/dev/ttyUSB0", 9600);

	tcgetattr(fd, &ttyparam);

	ttyparam.c_iflag &= ~IXON;
	ttyparam.c_iflag &= ~IXOFF;
	ttyparam.c_iflag &= ~CSIZE;
	ttyparam.c_iflag |= CS8;

	tcsetattr(fd, TCSANOW, &ttyparam);

	serialFlush(fd);



	int serial_rtn = 0;

	if(serial_rtn < 0){
		printf( "serial error¥n");
	}

}

/*
int Mg996rClass::EmergencyStop(){

	unsigned char write_data[5];
	int rtn = 0;


	write_data[0] = 0xFF;
	write_data[1] = 0x0b;
	write_data[2] = 0x00;
	write_data[3] = 0x01;
	write_data[4] = 0x00;

	rtn =  write(fd, write_data, sizeof(write_data));	

	return rtn;
}
*/

unsigned short Mg996rClass::TwopointInterpolation(double x, double a, double b){

	float tmp = 0;

	tmp = (float)(a * x + b);

	//RCLCPP_INFO(this->get_logger(), "tmp = %f", tmp);
	//RCLCPP_INFO(this->get_logger(), "tmp = %d", (unsigned short)tmp);

	return (unsigned short)tmp;
}


int Mg996rClass::SerialWrite(unsigned char write_data[], int len){

	int rtn = 0;


	rtn =  write(fd, write_data, len);	

	return rtn;
}


void Mg996rClass::Mg996rOperation(const mg996r_messages::msg::Mg996rMsg::SharedPtr msg){

	WITMOTION_COMMAND servo;
	unsigned short servo_speed = 0;
	unsigned short servo_position = 0;
	int serial_rtn = 0;


	msg->position += msg->offset;

	if(msg->position > 180.0)
		msg->position = 180.0;
	if(msg->position < 0.0)
		msg->position = 0.0;

	servo.speed_init_cmd = INIT_CMD;
	servo.speed_cmd = SPEED_CMD;
	servo.speed_ch = msg->ch;

	//servo_speed = msg->value_speed; 
	servo_speed = TwopointInterpolation(msg->value_speed, SPEED_SLOPE_A, SPEED_INTERCEPT_B);

	if(servo_speed < 1)
		servo_speed = 1;

	servo.speed_DataL = (unsigned char)(servo_speed & 0xFF);
	servo.speed_DataH = (unsigned char)((servo_speed >> 8)  & 0xFF);

	servo.position_init_cmd = INIT_CMD; 
	servo.position_cmd = POSITION_CMD;
	servo.position_ch = msg->ch;

	servo_position = TwopointInterpolation(msg->position, POSITION_SLOPE_A, POSITION_INTERCEPT_B);
	servo.position_DataL = (unsigned char)(servo_position & 0xFF);
	servo.position_DataH = (unsigned char)((servo_position >> 8) & 0xFF);


	RCLCPP_INFO(this->get_logger(), "ch = %d, msg->position = %f ,servo_position = %d", msg->ch, msg->position, servo_position);
	RCLCPP_INFO(this->get_logger(), "msg->speed = %f ,servo_speed = %d", msg->value_speed, servo_speed);
	
	serial_rtn = SerialWrite((unsigned char *)&servo, sizeof(servo));

	if(serial_rtn < 0){
		RCLCPP_INFO(this->get_logger(), "serial error");
	}
	
}


