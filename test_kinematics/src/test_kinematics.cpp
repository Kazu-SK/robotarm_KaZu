
#include "test_kinematics/test_kinematics.hpp"


void TestKinematics::TestKinematicsInitialize(){
/*
	target_x = 90.0;  
	target_y = 10.0; 
	target_z = 30.0; 
	target_pitch = 180.0 * M_PI / 180.0; 
	target_yaw = 0.0 * M_PI / 180.0; 
	*/


	//init parameter table
	robot_joint[SERVO_ID_0].channel_num = 0;
	robot_joint[SERVO_ID_0].value_speed = 6;
	robot_joint[SERVO_ID_0].offset = 90.0;
	robot_joint[SERVO_ID_0].log_position = 0.0;

	robot_joint[SERVO_ID_1].channel_num = 1;
	robot_joint[SERVO_ID_1].value_speed = 4;
	robot_joint[SERVO_ID_1].offset = 90.0; 
	robot_joint[SERVO_ID_1].log_position = 0.0;

	robot_joint[SERVO_ID_2].channel_num = 2;
	robot_joint[SERVO_ID_2].value_speed = 4;
	robot_joint[SERVO_ID_2].offset = 4.0;
	//robot_joint[SERVO_ID_2].offset = 8.0;
	robot_joint[SERVO_ID_2].log_position = 0.0;

	robot_joint[SERVO_ID_3].channel_num = 3;
	robot_joint[SERVO_ID_3].value_speed = 5;
	robot_joint[SERVO_ID_3].offset = 0.0;
	robot_joint[SERVO_ID_3].offset = 2.5;
	robot_joint[SERVO_ID_3].log_position = 0.0;

	robot_joint[SERVO_ID_4].channel_num = 4;
	robot_joint[SERVO_ID_4].value_speed = 4;
	robot_joint[SERVO_ID_4].offset = 90.0;
	robot_joint[SERVO_ID_4].log_position = 0.0;

	robot_joint[SERVO_ID_5].channel_num = 5;
	robot_joint[SERVO_ID_5].value_speed = 15;
	robot_joint[SERVO_ID_5].offset = 90.0;
	robot_joint[SERVO_ID_5].log_position = 0.0;


	servo_num = SERVO_NUM; 


	HandClose();
	InitPosition();

}


void TestKinematics::Request(){

	auto request = std::make_shared<kinematics_service::srv::InvKinematics::Request>();

	request->x = target_x;
	request->y = target_y;
	request->z = target_z;
	request->pitch = target_pitch;
	request->yaw = target_yaw;

	RCLCPP_INFO(this->get_logger(),"request");
	RCLCPP_INFO(this->get_logger(),"result");

	RCLCPP_INFO(this->get_logger(),"target_x = %f", target_x);
	RCLCPP_INFO(this->get_logger(),"target_y = %f", target_y);
	RCLCPP_INFO(this->get_logger(),"target_z = %f", target_z);
	

	while(!client->wait_for_service(1s)){

		if(!rclcpp::ok()){
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			break;
		}
		RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
	}
	
	auto result = client->async_send_request(request); 

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

	CalcSpeed();


	for(int id = 0 ; id < SERVO_NUM ; id++){

		robot_joint[id].log_position = robot_joint[id].position; 
	}

	//Publish();

}


void TestKinematics::CalcSpeed(){

	for(int id = 0 ; id < SERVO_NUM ; id++){

		robot_joint[id].diff_position = fabs(robot_joint[id].position - robot_joint[id].log_position); 
		robot_joint[id].value_speed = robot_joint[id].diff_position / 1.5;
		
	}

	
}


void TestKinematics::InitPosition(){

	static int loop_count = 0;
	

	switch(loop_count){

		case 0:
			target_x = 20.0;  
			target_y = 100.0; 
			target_z = 190.0; 
			target_pitch = 180.0 * M_PI / 180.0; 
			target_yaw = 0.0 * M_PI / 180.0;//-atan2(target_y, target_x); 
			break;
		case 1:
			target_x = 70.0;  
			target_y = 0.0; 
			target_z = 190.0; 
			target_pitch = 180.0 * M_PI / 180.0; 
			target_yaw = 0.0 * M_PI / 180.0;//-atan2(target_y, target_x); 
			/*
			target_x = 230.0;  
			target_y = -5.0; 
			target_z = 160.0; 
			target_pitch = 135 * M_PI / 180.0; 
			target_yaw = 40.0 * M_PI / 180.0; 
			*/
			break;
		case 2:
			target_x = 20.0;  
			target_y = -100.0; 
			target_z = 190.0; 
			target_pitch = 180.0 * M_PI / 180.0; 
			target_yaw = 0.0 * M_PI / 180.0;//-atan2(target_y, target_x); 

			break;
		default:
			loop_count = 0;
			break;

	}

	Request();

	//loop_count++;

	/*
	if(loop_count > 1){
		loop_count = 0;
	}
	*/
/*
	target_x = 10.0;  
	target_y = -90.0; 
	target_z = 160.0; 
	target_pitch = 180.0 * M_PI / 180.0; 
	target_yaw = 0.0 * M_PI / 180.0; 
	*/
/*	
	target_x = 130.0;  
	target_y = 130.0; 
	target_z = 85.0; 
	target_pitch = 180.0 * M_PI / 180.0; 
	target_yaw = 0.0 * M_PI / 180.0; 
	*/
	//Request();
}


void TestKinematics::HandClose(){

	robot_joint[SERVO_ID_5].position = 0.0;
}


void TestKinematics::HandOpen(){

	robot_joint[SERVO_ID_5].position = 70.0;
}


void TestKinematics::ActionMain(){


	//rclcpp::Rate loop_rate(100);


	//while(rclcpp::ok()){

	switch(ac){

		case ROBOT_INIT_POSITION:
			InitPosition();
		//	ac = SET;
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
		case NONE:
			break;
		default:
			break;

	}

	
	//loop_rate.sleep();


	//}


}



void TestKinematics::WorldCoordinateSub(const vision_interfaces::msg::WorldCoordinate::SharedPtr msg){

	world_x = msg->world_x;
	world_y = msg->world_y;
	world_z = msg->world_z;

}



void TestKinematics::Mg996rPub(){

	for(int id = SERVO_ID_0 ; id < SERVO_NUM ; id++){
		mg996r_message.ch = robot_joint[id].channel_num;
		mg996r_message.value_speed = robot_joint[id].value_speed;
		mg996r_message.position = robot_joint[id].position;
		mg996r_message.offset = robot_joint[id].offset;

		test_kinematics_pub->publish(mg996r_message);
	}

}
