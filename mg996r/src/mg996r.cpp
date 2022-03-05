
#include "mg996r/mg996r.hpp"


//Two-point interpolation 
const int Mg996rClass::LOW_PULSE = 544;
const int Mg996rClass::HIGH_PULSE = 2400;
const float Mg996rClass::LOW_POSITION = 0.0;
const float Mg996rClass::HIGH_POSITION = 180.0;

//WITMOTION module
const unsigned char Mg996rClass::INIT_CMD = 0xFF;
const unsigned char Mg996rClass::SPEED_CMD = 0x01;
const unsigned char Mg996rClass::POSITION_CMD = 0x02;


void Mg996rClass::Mg996rInitialize(){

	SLOPE_A = (HIGH_PULSE - LOW_PULSE) / (HIGH_POSITION - LOW_POSITION);	
	INTERCEPT_B = HIGH_PULSE - SLOPE_A * HIGH_POSITION;

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

int Mg996rClass::SerialWrite(unsigned char write_data[], int len){

	int rtn = 0;


	rtn =  write(fd, write_data, len);	

	return rtn;
}


void Mg996rClass::Mg996rOperation(const mg996r_messages::msg::Mg996rMsg::SharedPtr msg){

	Witmotion_command servo;
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

	servo_speed = msg->value_speed; 
	servo.speed_DataL = (unsigned char)(servo_speed & 0xFF);
	servo.speed_DataH = (unsigned char)((servo_speed >> 8)  & 0xFF);

	servo.position_init_cmd = INIT_CMD; 
	servo.position_cmd = POSITION_CMD;
	servo.position_ch = msg->ch;

	servo_position = TwopointInterpolation(msg->position);

	servo.position_DataL = (unsigned char)(servo_position & 0xFF);
	servo.position_DataH = (unsigned char)((servo_position >> 8) & 0xFF);


	RCLCPP_INFO(this->get_logger(), "ch = %d, msg->position = %f ,servo_position = %d", msg->ch, msg->position, servo_position);
	
	serial_rtn = SerialWrite((unsigned char *)&servo, sizeof(servo));

	if(serial_rtn < 0){
		RCLCPP_INFO(this->get_logger(), "serial error");
	}
	
}


