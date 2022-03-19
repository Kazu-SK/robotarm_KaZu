
#include "test_kinematics/test_kinematics.hpp"


void TestKinematics::TestKinematicsInitialize(){

	target_x = 90.0;  
	target_y = -110.0; 
	target_z = 30.0; 
	target_pitch = 180.0 * M_PI / 180.0; 
	target_yaw = 0.0 * M_PI / 180.0; 

	//init parameter table
	robot_joint[SERVO_ID_0].channel_num = 0;
	robot_joint[SERVO_ID_0].value_speed = 6;
	robot_joint[SERVO_ID_0].position = -45.0; 
	robot_joint[SERVO_ID_0].position = 0.0;
	robot_joint[SERVO_ID_0].offset = 90.0;

	robot_joint[SERVO_ID_1].channel_num = 1;
	robot_joint[SERVO_ID_1].value_speed = 4;
	robot_joint[SERVO_ID_1].position = -10.0;
	robot_joint[SERVO_ID_1].position = 0.0;
	robot_joint[SERVO_ID_1].offset = 90.0; 

	robot_joint[SERVO_ID_2].channel_num = 2;
	robot_joint[SERVO_ID_2].value_speed = 4;
	robot_joint[SERVO_ID_2].position = 90.0;
	//robot_joint[SERVO_ID_2].position = 0.0;
	robot_joint[SERVO_ID_2].offset = 12.8;

	robot_joint[SERVO_ID_3].channel_num = 3;
	robot_joint[SERVO_ID_3].value_speed = 5;
	//robot_joint[SERVO_ID_3].position = 45.0;
	robot_joint[SERVO_ID_3].position = 90.0;
	//robot_joint[SERVO_ID_3].position = 0.0;
	robot_joint[SERVO_ID_3].offset = 90.0;

	robot_joint[SERVO_ID_4].channel_num = 4;
	robot_joint[SERVO_ID_4].value_speed = 4;
	robot_joint[SERVO_ID_4].position = -45.0;
	robot_joint[SERVO_ID_4].position = 0.0;
	robot_joint[SERVO_ID_4].offset = 90.0;

	robot_joint[SERVO_ID_5].channel_num = 5;
	robot_joint[SERVO_ID_5].value_speed = 2;
	robot_joint[SERVO_ID_5].position = 60.0;
	robot_joint[SERVO_ID_5].position = 0.0;
	robot_joint[SERVO_ID_5].offset = 90.0;


	servo_num = SERVO_NUM; 

}


void TestKinematics::Request(){

	auto request = std::make_shared<kinematics_service::srv::InvKinematics::Request>();

	request->x = target_x;
	request->y = target_y;
	request->z = target_z;
	request->pitch = target_pitch;
	request->yaw = target_yaw;

	RCLCPP_INFO(this->get_logger(),"request");
	auto result = client->async_send_request(request); 
	RCLCPP_INFO(this->get_logger(),"result");

	if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS){
		
		RCLCPP_INFO(this->get_logger(), "link1_q = %f",result.get()->link1_q * 180.0 / M_PI);
		RCLCPP_INFO(this->get_logger(), "link2_q = %f",result.get()->link2_q * 180.0 / M_PI);
		RCLCPP_INFO(this->get_logger(), "link3_q = %f",result.get()->link3_q * 180.0 / M_PI);
		RCLCPP_INFO(this->get_logger(), "link4_q = %f",result.get()->link4_q * 180.0 / M_PI);
		RCLCPP_INFO(this->get_logger(), "link5_q = %f",result.get()->link5_q * 180.0 / M_PI);
		

		robot_joint[SERVO_ID_0].position = result.get()->link1_q * 180.0 / M_PI; 
		robot_joint[SERVO_ID_1].position = result.get()->link2_q * 180.0 / M_PI; 
		robot_joint[SERVO_ID_2].position = result.get()->link3_q * 180.0 / M_PI; 
		robot_joint[SERVO_ID_3].position = result.get()->link4_q * 180.0 / M_PI; 
		robot_joint[SERVO_ID_4].position = result.get()->link5_q * 180.0 / M_PI; 

	}
	else{

		RCLCPP_ERROR(this->get_logger(), "Failed");
	}
}

void TestKinematics::Publish(){

	for(int id = SERVO_ID_0 ; id < SERVO_NUM ; id++){
		mg996r_message.ch = robot_joint[id].channel_num;
		mg996r_message.value_speed = robot_joint[id].value_speed;
		mg996r_message.position = robot_joint[id].position;
		mg996r_message.offset = robot_joint[id].offset;

		test_operation_pub->publish(mg996r_message);
	}

}


void TestKinematics::ActionMain(){


	//rclcpp::Rate loop_rate(100);


	//while(rclcpp::ok()){
	RCLCPP_INFO(this->get_logger(),"testkinematics");

	switch(ac){

		case ROBOT_INIT_POSITION:
			RCLCPP_INFO(this->get_logger(),"testkinematics");
			Request();
			break;
		case SET:
			break;
		case CLOSER:
			break;
		case LIFT:
			break;
		case MOVE:
			break;
		case PUT:
			break;
		case HAND_OPEN:
			break;
		case HAND_CLOSE:
			break;

	}

	
	//loop_rate.sleep();


	//}


}
