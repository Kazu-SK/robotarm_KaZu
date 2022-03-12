
#include "robot_data/robot_data.hpp"


void RobotData::Initialize(){

	std::cout<<"robotdata_initialize"<<std::endl;

	//double UX[3] = {1.0, 0.0, 0.0};
	double UY[3] = {0.0, 1.0, 0.0};
	double UZ[3] = {0.0, 0.0, 1.0}; 

	/*   Link information  */
	strcpy(ulink[ULINK_ID_1].name, "Base");  
	ulink[ULINK_ID_1].sister = 0;
	ulink[ULINK_ID_1].child = ULINK_ID_2;  
	ulink[ULINK_ID_1].p[0] = 0.0;
	ulink[ULINK_ID_1].p[1] = 0.0;
	ulink[ULINK_ID_1].p[2] = 0.0;
	ulink[ULINK_ID_1].q = 45.0 * M_PI/180.0;
	matrix->SubstituteMatrix31(ulink[ULINK_ID_1].a, UZ);
	matrix->Yaw(ulink[ULINK_ID_1].R,ulink[ULINK_ID_1].q); 

	strcpy(ulink[ULINK_ID_2].name, "LINK1");  
	ulink[ULINK_ID_2].sister = 0;
	ulink[ULINK_ID_2].child = ULINK_ID_3;  
	ulink[ULINK_ID_2].b[0] = 0.0; 
	ulink[ULINK_ID_2].b[1] = 0.0; 
	ulink[ULINK_ID_2].b[2] = 220.0; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_2].a, UY);
	ulink[ULINK_ID_2].q = -90.0 * M_PI/180.0;

	strcpy(ulink[ULINK_ID_3].name, "LINK2");  
	ulink[ULINK_ID_3].sister = 0;
	ulink[ULINK_ID_3].child = ULINK_ID_4;
	ulink[ULINK_ID_3].b[0] = 0.0; 
	ulink[ULINK_ID_3].b[1] = 0.0; 
	ulink[ULINK_ID_3].b[2] = 95.0; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_3].a, UY);
	ulink[ULINK_ID_3].q = 90.0 * M_PI/180.0;

	strcpy(ulink[ULINK_ID_4].name, "LINK3");  
	ulink[ULINK_ID_4].sister = 0;
	ulink[ULINK_ID_4].child = ULINK_ID_5;
	ulink[ULINK_ID_4].b[0] = 0.0; 
	ulink[ULINK_ID_4].b[1] = 0.0; 
	ulink[ULINK_ID_4].b[2] = 95.0; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_4].a, UY);
	ulink[ULINK_ID_4].q = 90.0 * M_PI/180.0;

	strcpy(ulink[ULINK_ID_5].name, "LINK4");  
	ulink[ULINK_ID_5].sister = 0;
	ulink[ULINK_ID_5].child = ULINK_ID_6;
	ulink[ULINK_ID_5].b[0] = 0.0; 
	ulink[ULINK_ID_5].b[1] = 0.0; 
	ulink[ULINK_ID_5].b[2] = 77.75; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_5].a, UZ);
	ulink[ULINK_ID_5].q = 45.0 * M_PI/180.0;

	strcpy(ulink[ULINK_ID_6].name, "LINK5");  
	ulink[ULINK_ID_6].sister = 0;
	ulink[ULINK_ID_6].child = 0;
	ulink[ULINK_ID_6].b[0] = 0.0; 
	ulink[ULINK_ID_6].b[1] = 0.0; 
	ulink[ULINK_ID_6].b[2] = 67.25; 

	FindMother(ULINK_ID_1);
}


void RobotData::FindMother(int j){

	if(j != 0){

		if(j == 1){
			ulink[j].mother = 0;
		}

		if(ulink[j].child != 0){
			ulink[ulink[j].child].mother = j;
			//RCLCPP_INFO(this->get_logger(),"ulink[%d].mother = %d",ulink[j].child,ulink[ulink[j].child].mother);
			FindMother(ulink[j].child);

		}

		if(ulink[j].sister != 0){
			ulink[ulink[j].sister].mother = ulink[j].mother;
			FindMother(ulink[j].sister);
		}

	}

}


void RobotData::ForwardKinematics(int j){

	if(j == 0) return;

	if(j != 1){

		double Rod[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
		double val[3] = {0, 0, 0};
		int mom = 0;

		mom = ulink[j].mother;
		matrix->MultiMatrix31(val, ulink[mom].R, ulink[j].b);

		/*
		RCLCPP_INFO(this->get_logger(),"id = %d",j);
		for(int z = 0 ; z < 3 ; z++){
			RCLCPP_INFO(this->get_logger(),"%f %f %f",ulink[mom].R[z][0],ulink[mom].R[z][1],ulink[mom].R[z][2]);
		}
		for(int z = 0 ; z < 3 ; z++){
			RCLCPP_INFO(this->get_logger(),"val[%d] = %f",z,val[z]);
		}
		*/
		matrix->SumMatrix31(ulink[j].p, val, ulink[mom].p);

		Rodrigues(Rod, ulink[j].a, ulink[j].q);
		/*
		RCLCPP_INFO(this->get_logger(),"id = %d",j);
		for(int z = 0 ; z < 3 ; z++){
			RCLCPP_INFO(this->get_logger(),"%f %f %f",Rod[z][0],Rod[z][1],Rod[z][2]);
		}
		*/

		matrix->InitMatrix33(ulink[j].R);
		matrix->MultiMatrix33(ulink[j].R, ulink[mom].R, Rod);
	}

	ForwardKinematics(ulink[j].sister);
	ForwardKinematics(ulink[j].child);
}	


void RobotData::Rodrigues(double R[3][3], double w[3], double dt){

	double th = 0.0;
	double wn[3] = {0, 0, 0};
	double multi_wedge[3][3] = {{0, 0, 0},{0, 0, 0},{0, 0, 0}};
	double eye[3][3] = {{1, 0, 0},{0, 1, 0},{0, 0, 1}};

	

	th = matrix->Norm31(w) * dt;

	wn[0] = w[0]/matrix->Norm31(w);
	wn[1] = w[1]/matrix->Norm31(w);
	wn[2] = w[2]/matrix->Norm31(w);

	double w_wedge[3][3] = {
		{0	,-wn[2] 	,wn[1]},
		{wn[2]	,0		,-wn[0]},
		{-wn[1]	,wn[0]	,0    }
	};

	matrix->MultiMatrix33(multi_wedge, w_wedge, w_wedge);

	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 3 ; j++){

			R[i][j] = eye[i][j] + w_wedge[i][j] * sin(th) + multi_wedge[i][j] * (1 - cos(th)); 	
		}
	}
	
}

void RobotData::TestPublish(){

	//RCLCPP_INFO(this->getlogger(),"*** position ***");
	
	/*
	for(int i = 1 ; i < ULINK_NUM ; i++){
		RCLCPP_INFO(this->get_logger(),"ulink[%d].mother = %d",i,ulink[i].mother);
	}
	*/

	ForwardKinematics(ULINK_ID_1);
	

	for(int j = 0 ; j < ULINK_NUM ; j++){
		RCLCPP_INFO(this->get_logger(),"ulink[%d].name = %s",j,ulink[j].name);
		for(int i = 0 ; i < 3 ; i++){
			RCLCPP_INFO(this->get_logger(),"ulink[%d].p[%d] = %f",j,i,ulink[j].p[i]);
		}
	}
	
}
