
#include "test_operation/test_operation.hpp"




void TestOperation::TestOperationInitialize(){

	//init parameter table
	robot_joint[SERVO_ID_0].channel_num = 0;
	robot_joint[SERVO_ID_0].value_speed = 6;
	robot_joint[SERVO_ID_0].position = -45.0; 
	robot_joint[SERVO_ID_0].position = 0.0;
	robot_joint[SERVO_ID_0].offset = 0.0;

	robot_joint[SERVO_ID_1].channel_num = 1;
	robot_joint[SERVO_ID_1].value_speed = 4;
	robot_joint[SERVO_ID_1].position = -10.0;
	robot_joint[SERVO_ID_1].position = 0.0;
	robot_joint[SERVO_ID_1].offset = 90.0; 

	robot_joint[SERVO_ID_2].channel_num = 2;
	robot_joint[SERVO_ID_2].value_speed = 4;
	robot_joint[SERVO_ID_2].position = 90.0;
	robot_joint[SERVO_ID_2].position = 0.0;
	robot_joint[SERVO_ID_2].offset = 0.0;

	robot_joint[SERVO_ID_3].channel_num = 3;
	robot_joint[SERVO_ID_3].value_speed = 5;
	robot_joint[SERVO_ID_3].position = 45.0;
	robot_joint[SERVO_ID_3].position = 0.0;
	robot_joint[SERVO_ID_3].offset = 90.0;

	robot_joint[SERVO_ID_4].channel_num = 4;
	robot_joint[SERVO_ID_4].value_speed = 4;
	robot_joint[SERVO_ID_4].position = -45.0;
	robot_joint[SERVO_ID_4].position = 90.0;
	robot_joint[SERVO_ID_4].offset = 90.0;

	robot_joint[SERVO_ID_5].channel_num = 5;
	robot_joint[SERVO_ID_5].value_speed = 2;
	robot_joint[SERVO_ID_5].position = 60.0;
	robot_joint[SERVO_ID_5].position = 0.0;
	robot_joint[SERVO_ID_5].offset = 90.0;


	servo_num = SERVO_NUM; 
}


void TestOperation::Publish(){

	for(int id = SERVO_ID_0 ; id < SERVO_NUM ; id++){
		mg996r_message.ch = robot_joint[id].channel_num;
		mg996r_message.value_speed = robot_joint[id].value_speed;
		mg996r_message.position = robot_joint[id].position;
		mg996r_message.offset = robot_joint[id].offset;

		test_operation_pub->publish(mg996r_message);
	}

}





