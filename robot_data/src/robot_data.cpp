
#include "robot_data/robot_data.hpp"


const double RobotData::FX = 698.2863954;
const double RobotData::FY = 696.32192016;
const double RobotData::CX = 316.8553548;
const double RobotData::CY = 233.13340412; 

const double RobotData::OBJECT_HEIGHT = 0.0; //[mm]
const double RobotData::PIXEL_SIZE = 0.006; //[mm] 
const double RobotData::IMAGE_AREA_X = 3.984; //[mm]
const double RobotData::IMAGE_AREA_Y = 2.952; //[mm]
const int RobotData::REVOLUSION_H = 640;
const int RobotData::REVOLUSION_V = 480;


void RobotData::Initialize(){

	//double UX[3] = {1.0, 0.0, 0.0};
	double UY[3] = {0.0, 1.0, 0.0};
	double UZ[3] = {0.0, 0.0, 1.0}; 

	strcpy(ulink[ULINK_ID_T].name,"Target");

	strcpy(ulink[ULINK_ID_1].name, "Base");  
	ulink[ULINK_ID_1].sister = 0;
	ulink[ULINK_ID_1].child = ULINK_ID_2;  
	ulink[ULINK_ID_1].p[0] = 0.0;
	ulink[ULINK_ID_1].p[1] = 0.0;
	ulink[ULINK_ID_1].p[2] = 0.0;
	ulink[ULINK_ID_1].q = 0.0;
	matrix->SubstituteMatrix31(ulink[ULINK_ID_1].a, UZ);
	matrix->Yaw(ulink[ULINK_ID_1].R,ulink[ULINK_ID_1].q); 

	/*   Link information  */
	strcpy(ulink[ULINK_ID_2].name, "LINK1");  
	ulink[ULINK_ID_2].sister = 0;
	ulink[ULINK_ID_2].child = ULINK_ID_3;  
	ulink[ULINK_ID_2].b[0] = 0.0;
	ulink[ULINK_ID_2].b[1] = 0.0;
	ulink[ULINK_ID_2].b[2] = 170.2;
	ulink[ULINK_ID_2].q = 45.0 * M_PI/180.0;
	matrix->SubstituteMatrix31(ulink[ULINK_ID_2].a, UZ);
	matrix->Yaw(ulink[ULINK_ID_2].R,ulink[ULINK_ID_2].q); 

	strcpy(ulink[ULINK_ID_3].name, "LINK2");  
	ulink[ULINK_ID_3].sister = 0;
	ulink[ULINK_ID_3].child = ULINK_ID_4;  
	ulink[ULINK_ID_3].b[0] = 0.0; 
	ulink[ULINK_ID_3].b[1] = 0.0; 
	ulink[ULINK_ID_3].b[2] = 29.8; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_3].a, UY);
	//ulink[ULINK_ID_3].q = -90.0 * M_PI/180.0;
	ulink[ULINK_ID_3].q = -30.0 * M_PI/180.0;

	strcpy(ulink[ULINK_ID_4].name, "LINK3");  
	ulink[ULINK_ID_4].sister = 0;
	ulink[ULINK_ID_4].child = ULINK_ID_5;
	ulink[ULINK_ID_4].b[0] = 0.0; 
	ulink[ULINK_ID_4].b[1] = 0.0; 
	ulink[ULINK_ID_4].b[2] = 95.0; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_4].a, UY);
	ulink[ULINK_ID_4].q = 80.0 * M_PI/180.0;

	strcpy(ulink[ULINK_ID_5].name, "LINK4");  
	ulink[ULINK_ID_5].sister = 0;
	ulink[ULINK_ID_5].child = ULINK_ID_6;
	ulink[ULINK_ID_5].b[0] = 0.0; 
	ulink[ULINK_ID_5].b[1] = 0.0; 
	ulink[ULINK_ID_5].b[2] = 95.0; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_5].a, UY);
	ulink[ULINK_ID_5].q = 80.0 * M_PI/180.0;

	strcpy(ulink[ULINK_ID_6].name, "LINK5");  
	ulink[ULINK_ID_6].sister = 0;
	//ulink[ULINK_ID_6].child = ULINK_ID_6;
	ulink[ULINK_ID_6].child = ULINK_ID_CAMERA;
	ulink[ULINK_ID_6].b[0] = 0.0; 
	ulink[ULINK_ID_6].b[1] = 0.0; 
	//ulink[ULINK_ID_6].b[2] = 77.75; 
	ulink[ULINK_ID_6].b[2] = 145.0; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_6].a, UZ);
	ulink[ULINK_ID_6].q = 45.0 * M_PI/180.0;


	strcpy(ulink[ULINK_ID_CAMERA].name, "LINK_CAMERA");  
	ulink[ULINK_ID_CAMERA].sister = 0;
	ulink[ULINK_ID_CAMERA].child = 0;
	ulink[ULINK_ID_CAMERA].b[0] = -63.95; 
	ulink[ULINK_ID_CAMERA].b[1] = 2.4; 
	//ulink[ULINK_ID_CAMERA].b[2] = 101.35; 
	ulink[ULINK_ID_CAMERA].b[2] = -43.65; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_CAMERA].a, UZ);
	ulink[ULINK_ID_CAMERA].q = -90.0 * M_PI/180.0;
	/*
	strcpy(ulink[ULINK_ID_6].name, "LINK5");  
	ulink[ULINK_ID_6].sister = 0;
	ulink[ULINK_ID_6].child = 0;
	ulink[ULINK_ID_6].b[0] = 0.0; 
	ulink[ULINK_ID_6].b[1] = 0.0; 
	ulink[ULINK_ID_6].b[2] = 67.25; 
	*/

	FindMother(ULINK_ID_1);

	//world_coordinate[0] = 999.0;
	//world_coordinate[1] = 999.0;
	//world_coordinate[2] = 999.0;
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

		//matrix->InitMatrix33(ulink[j].R);
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


void RobotData::InverseKinematicsNum(double p[3], double pitch, double yaw){
	
	double pitch_R[3][3] = {0.0, 0.0, 0.0};
	double yaw_R[3][3] = {0.0, 0.0, 0.0};
	double lambda = 0.9;
	int loop_count = 20;
	double lambda_add = lambda / loop_count; 
	double Jacobian[5][5];
	double inv_Jacobian[5][5];
	double err[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
	double tmp[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
	double dq[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
	int index = 0;


	matrix->InitMatrix55(Jacobian);
	matrix->InitMatrix55(inv_Jacobian);

	matrix->Pitch(pitch_R, pitch);
	matrix->Yaw(yaw_R, yaw);

	matrix->MultiMatrix33(ulink[ULINK_ID_T].R, pitch_R, yaw_R);
	for(int i = 0 ; i < 3 ; i++)ulink[ULINK_ID_T].p[i] = p[i]; 
	ForwardKinematics(ULINK_ID_1);

	//RCLCPP_INFO(this->get_logger(),"init");
	/*
	for(int i = 0 ; i < 3 ; i++){
		RCLCPP_INFO(this->get_logger(),"ulink[ULINK_ID_6].p[%d] = %f",i,ulink[ULINK_ID_6].p[i]);
	}
	*/
	
	//int n = 0;
	for(int n = 1 ; n < loop_count ; n++){
	//for(;;){

		CalcJacobian(Jacobian);

		CalcVWerr(err);

		if(matrix->Norm51(err) < 0.3)
			break;



		matrix->InverseMatrix55(inv_Jacobian, Jacobian);
		matrix->MultiMatrix51(tmp, inv_Jacobian, err);

		for(int j = 0 ; j < 5 ; j++){
			dq[j] = lambda * tmp[j];
		}

		index = 0;
		for(int j = ULINK_ID_2 ; j < ULINK_INDEX_NUM ; j++){
			ulink[j].q += dq[index];	
			index++;
		}

		ForwardKinematics(ULINK_ID_1);

		//RCLCPP_INFO(this->get_logger(),"n = %d",n);
		//RCLCPP_INFO(this->get_logger(),"loop");
		/*
		for(int i = 0 ; i < 3 ; i++){
			RCLCPP_INFO(this->get_logger(),"ulink[ULINK_ID_6].p[%d] = %f",i,ulink[ULINK_ID_6].p[i]);
		}
		*/

		lambda += lambda_add;

	//	n++;
	}
	

	/*
	CalcJacobian(Jacobian);
	matrix->InverseMatrix55(inv_Jacobian, Jacobian);

	matrix->MultiMatrix55(tmp, Jacobian, inv_Jacobian);
	*/
}


void RobotData::CalcJacobian(double J[5][5]){

	double a[3] = {0.0, 0.0, 0.0};
	double cross[3] = {0.0, 0.0, 0.0};
	double perr[3] = {0.0, 0.0, 0.0};
	int j_col = 0;

/*
	for(int i = 0 ; i < 3 ; i++)perr[i] = ulink[ULINK_ID_6].p[i] - ulink[ULINK_ID_6].p[i];

	*/
	for(int j = ULINK_ID_2 ; j < ULINK_INDEX_NUM ; j++){

		matrix->MultiMatrix31(a, ulink[j].R, ulink[j].a);

		/*	
		std::cout<<a[0]<<std::endl;
		std::cout<<a[1]<<std::endl;
		std::cout<<a[2]<<std::endl;
		*/
		for(int i = 0 ; i < 3 ; i++){
			perr[i] = ulink[ULINK_ID_6].p[i] - ulink[j].p[i];
		}
	
		matrix->CrossMatrix31(cross, a, perr);

		J[0][j_col] = cross[0]; 	
		J[1][j_col] = cross[1]; 	
		J[2][j_col] = cross[2]; 	
		J[3][j_col] = a[1];
		J[4][j_col] = a[2];

		j_col++;
	}

	/*
	for(int y = 0 ; y < 5 ; y++){

		for(int z = 0 ; z < 5 ; z++){
			std::cout<<J[y][z]<<"  ";
		}
		std::cout<<std::endl;
	}
	*/
}


void RobotData::CalcVWerr(double e[5]){

	double perr[3] = {0.0, 0.0, 0.0};

	double inv_R[3][3] = {{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},{0.0,0.0,0.0}};
	double Rerr[3][3] = {{0.0, 0.0, 0.0},{0.0, 0.0, 0.0},{0.0,0.0,0.0}};

	double rot2_omega[3] = {0.0, 0.0, 0.0};
	double werr[3] = {0.0, 0.0, 0.0};


	for(int i = 0 ; i < 3 ; i++)perr[i] = ulink[ULINK_ID_T].p[i] - ulink[ULINK_ID_6].p[i]; 
	
	matrix->InverseMatrix33(inv_R, ulink[ULINK_ID_6].R);

	matrix->MultiMatrix33(Rerr, inv_R, ulink[ULINK_ID_T].R);
	
	Rot2Omega(rot2_omega, Rerr); 
	matrix->MultiMatrix31(werr, ulink[ULINK_ID_6].R, rot2_omega);

	for(int j = 0 ; j < 3 ; j++)e[j] = perr[j];	

	e[3] = werr[1];
	e[4] = werr[2]; 

}


void RobotData::Rot2Omega(double w[3], double R[3][3]){

	double alpha = 0.0;
	double EPS = 2.22e-16;
	double th = 0.0;


	alpha = matrix->TraceMatrix33(R);
	alpha = (alpha - 1.0) / 2.0;


	if(fabs(alpha-1.0) < EPS){

		w[0] = 0.0;
		w[1] = 0.0;
		w[2] = 0.0;

		return; 
	}	

	th = acos(alpha);

	w[0] = 0.5 * th / sin(th) * (R[2][1] - R[1][2]);
	w[1] = 0.5 * th / sin(th) * (R[0][2] - R[2][0]);
	w[2] = 0.5 * th / sin(th) * (R[1][0] - R[0][1]);

}


void RobotData::InverseKinematicsAna(double p[3], double pitch, double yaw){

	double buff3[3] = {0.0, 0.0, 0.0};
	double buff4[3] = {0.0, 0.0, 0.0};
	double RY[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double RZ[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double R1[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double R1_T[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double R2[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double R2_T[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double R3[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double R3_T[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double buff[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
	double buff2[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};

	double link4_p[3] = {0.0, 0.0, 0.0};	

	double link12_l_sum = ulink[ULINK_ID_2].b[2] + ulink[ULINK_ID_3].b[2];

	double phi = 0.0;
	double P1P = 0.0;

	double cos_beta = 0.0;
	double sin_beta = 0.0;
	double beta = 0.0;

	double cos_alpha = 0.0;
	double sin_alpha = 0.0;
	double alpha = 0.0;

	double temp_z_angle = 0.0;

	temp_z_angle = atan2(p[1], p[0]);

	matrix->Pitch(RY, pitch); 
	matrix->Yaw(RZ, yaw); 

	matrix->MultiMatrix31(buff3, RZ, ulink[ULINK_ID_6].b); 
	matrix->MultiMatrix31(buff4, RY, buff3); 
	matrix->Yaw(RZ, temp_z_angle);
	matrix->MultiMatrix31(buff3, RZ, buff4); 

	link4_p[0] = p[0] - buff3[0]; 
	link4_p[1] = p[1] - buff3[1]; 
	link4_p[2] = p[2] - buff3[2]; 

	matrix->Yaw(RZ, yaw); 

	ulink[ULINK_ID_2].q = atan2(link4_p[1], link4_p[0]);

	phi = atan2(link4_p[2] - link12_l_sum, sqrt(link4_p[0]*link4_p[0] + link4_p[1]*link4_p[1]));
	P1P = sqrt(link4_p[0]*link4_p[0] + link4_p[1]*link4_p[1] + (link4_p[2] - link12_l_sum) * (link4_p[2] - link12_l_sum));

	cos_alpha = (ulink[ULINK_ID_4].b[2] * ulink[ULINK_ID_4].b[2] + P1P * P1P - ulink[ULINK_ID_5].b[2] * ulink[ULINK_ID_5].b[2]) / (2.0 * ulink[ULINK_ID_4].b[2] * P1P);
	sin_alpha = sqrt(1.0 - cos_alpha * cos_alpha); 
	alpha = atan2(sin_alpha, cos_alpha);	

	cos_beta = (ulink[ULINK_ID_4].b[2] * ulink[ULINK_ID_4].b[2] + ulink[ULINK_ID_5].b[2] * ulink[ULINK_ID_5].b[2] - P1P * P1P) / (2 * ulink[ULINK_ID_4].b[2] * ulink[ULINK_ID_5].b[2]); 
	sin_beta = sqrt(1.0 - cos_beta * cos_beta);
	beta = atan2(sin_beta, cos_beta);

	ulink[ULINK_ID_3].q = M_PI / 2.0 - phi - alpha;
	ulink[ULINK_ID_4].q = M_PI - beta;

	matrix->Yaw(R1, ulink[ULINK_ID_2].q); 
	matrix->Pitch(R2, ulink[ULINK_ID_3].q); 
	matrix->Pitch(R3, ulink[ULINK_ID_4].q); 
	matrix->TransposeMatrix33(R1_T, R1);  
	matrix->TransposeMatrix33(R2_T, R2);  
	matrix->TransposeMatrix33(R3_T, R3);  
	matrix->MultiMatrix33(buff, RY, RZ); 
	matrix->MultiMatrix33(buff2, R1_T, buff); 
	matrix->MultiMatrix33(buff, R2_T, buff2);
	matrix->MultiMatrix33(buff2, R3_T, buff);

	ulink[ULINK_ID_5].q = atan2(buff2[0][2], buff2[2][2]);
	ulink[ULINK_ID_6].q = atan2(buff2[2][1], -buff2[2][0]);


	ForwardKinematics(ULINK_ID_1);
}



void RobotData::InvKinemaService(const std::shared_ptr<kinematics_service::srv::InvKinematics::Request> request,std::shared_ptr<kinematics_service::srv::InvKinematics::Response> response){

	double target_position[3] = {0.0, 0.0, 0.0};

	target_position[0] = request->x;
	target_position[1] = request->y;
	target_position[2] = request->z;

	InverseKinematicsAna(target_position, request->pitch, request->yaw); 

	response->link1_q = ulink[ULINK_ID_2].q;
	response->link2_q = ulink[ULINK_ID_3].q;
	response->link3_q = ulink[ULINK_ID_4].q;
	response->link4_q = ulink[ULINK_ID_5].q;
	response->link5_q = ulink[ULINK_ID_6].q;
}



//void RobotData::CoordinateConversionService(const std::shared_ptr<vision_service::srv::CoordinateConversion::Request> request,std::shared_ptr<vision_service::srv::CoordinateConversion::Response> response){


void RobotData::CoordinateConversion(const vision_interfaces::msg::ImageCoordinate::SharedPtr msg){

	/***  image_coordinate --> camera_coordinate  ***/

	double camera_matrix[3][3] = {
		{FX, 0.0, CX},
		{0.0, FY, CY},
		{0.0, 0.0, 1.0}	
	};

	double image_coordinate[3] = {
		(double)msg->image_u, 
		(double)msg->image_v, 
		1.0
	};


	double camera_coordinate[3] = {0.0, 0.0, 0.0};

	double inv_matrix[3][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};

	//double fx_mm = 0.0;
	//double fy_mm = 0.0;

	double s = ulink[ULINK_ID_CAMERA].p[2] / PIXEL_SIZE; 


	for(int i = 0 ; i < 3 ; i++){
		image_coordinate[i] *= s;
	}
	
	
/*	
	RCLCPP_INFO(this->get_logger(), "image_u = %f", (double)msg->image_u);
	RCLCPP_INFO(this->get_logger(), "image_v = %f", (double)msg->image_v);
	*/
	
	//ulink[ULINK_ID_CAMERA].q = ulink[ULINK_ID_6].q + (90.0 * M_PI/180.0);
	
	ulink[ULINK_ID_CAMERA].q = -90.0 * M_PI/180.0;
	ForwardKinematics(ULINK_ID_1);

	matrix->InverseMatrix33(inv_matrix,camera_matrix);//, inv_matrix);
	matrix->MultiMatrix31(camera_coordinate, inv_matrix, image_coordinate);

	//fx_mm = IMAGE_AREA_X / REVOLUSION_H * FX; 
	//fy_mm = IMAGE_AREA_Y / REVOLUSION_V * FY; 
	
	for(int i = 0 ; i < 3 ; i++){
		//camera_coordinate[i] *= PIXEL_SIZE;
		//camera_coordinate[i] *= (ulink[ULINK_ID_CAMERA].p[2] - OBJECT_HEIGHT)/PIXEL_SIZE;
		camera_coordinate[i] *= ulink[ULINK_ID_CAMERA].p[2] - OBJECT_HEIGHT;
	}
		
	
/*	
	RCLCPP_INFO(this->get_logger(), "camera_coordinate[0] = %f", camera_coordinate[0]);
	RCLCPP_INFO(this->get_logger(), "camera_coordinate[1] = %f", camera_coordinate[1]);
	RCLCPP_INFO(this->get_logger(), "camera_coordinate[2] = %f", camera_coordinate[2]);
	*/
	
	
	//camera_coordinate[i] *= OBJECT_HEIGHT;



	
	/***  camera_coordinate --> world_coordinate  ***/

	double R1[3][3];

	double R1_T[3][3];
	double R2[3][3];
	double R2_T[3][3];
	double R3[3][3];
	double R3_T[3][3];
	double R4[3][3];
	double R4_T[3][3];
	double R5[3][3];
	double R5_T[3][3];
	double RC[3][3];
	double RC_T[3][3];
	double buff[3];
	double buff2[3];
	double origin_alignment[3];
	//double world_coordinate[3];


	for(int i = 0 ; i < 3 ; i++){
		origin_alignment[i] = camera_coordinate[i] - ulink[ULINK_ID_CAMERA].p[i];
	}	


	matrix->Yaw(RC, ulink[ULINK_ID_CAMERA].q); 
	matrix->InitMatrix33(RC_T);
	matrix->TransposeMatrix33(RC_T, RC);  

	matrix->Yaw(R5, ulink[ULINK_ID_6].q); 
	matrix->InitMatrix33(R5_T);
	matrix->TransposeMatrix33(R5_T, R5);  

	matrix->Pitch(R4, ulink[ULINK_ID_5].q); 
	matrix->InitMatrix33(R4_T);
	matrix->TransposeMatrix33(R4_T, R4);  

	matrix->Pitch(R3, ulink[ULINK_ID_4].q); 
	matrix->InitMatrix33(R3_T);
	matrix->TransposeMatrix33(R3_T, R3);  

	matrix->Pitch(R2, ulink[ULINK_ID_3].q); 
	matrix->InitMatrix33(R2_T);
	matrix->TransposeMatrix33(R2_T, R2);  

	matrix->Yaw(R1, ulink[ULINK_ID_2].q); 
	matrix->InitMatrix33(R1_T);
	matrix->TransposeMatrix33(R1_T, R1);  

	
	matrix->MultiMatrix31(buff, R1_T, origin_alignment);
	matrix->MultiMatrix31(buff2, R2_T, buff);
	matrix->MultiMatrix31(buff, R3_T, buff2);
	matrix->MultiMatrix31(buff2, R4_T, buff);
	matrix->MultiMatrix31(buff, R5_T, buff2);
	matrix->MultiMatrix31(world_coordinate, RC_T, buff);

	
	/*
	for(int i = 0 ; i < 3 ; i++){
		RCLCPP_INFO(this->get_logger(), "world_coordinate[%d] = %f",i, world_coordinate[i]);
	}
	*/
	

	
	
	
	matrix->InitMatrix33(inv_matrix);
	matrix->InverseMatrix33(inv_matrix, ulink[ULINK_ID_CAMERA].R);//, inv_matrix);
	matrix->MultiMatrix31(world_coordinate, inv_matrix, origin_alignment);
	
	
//	matrix->MultiMatrix31(origin_alignment, inv_matrix, world_coordinate);
/*	
	for(int i = 0 ; i < 3 ; i++){
		RCLCPP_INFO(this->get_logger(), "world_coordinate[%d] = %f",i, world_coordinate[i]);
	}
	*/


	/*
	response->world_x = world_coordinate[0];
	response->world_y = world_coordinate[1];
	response->world_z = world_coordinate[2];
	*/

	
	
	
	//world_coordinate_msg.world_x = world_coordinate[0];
	//world_coordinate_msg.world_y = world_coordinate[1];
	//world_coordinate_msg.world_z = world_coordinate[2];
	
	
        //world_coordinate_pub->publish(world_coordinate_msg);	
	

}


/*
void RobotData::pub_callback(){


}
*/

