
#include "robot_data/robot_data.hpp"


void RobotData::Initialize(){

	std::cout<<"robotdata_initialize"<<std::endl;

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
	ulink[ULINK_ID_6].child = 0;
	ulink[ULINK_ID_6].b[0] = 0.0; 
	ulink[ULINK_ID_6].b[1] = 0.0; 
	//ulink[ULINK_ID_6].b[2] = 77.75; 
	ulink[ULINK_ID_6].b[2] = 145.0; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_6].a, UZ);
	ulink[ULINK_ID_6].q = 45.0 * M_PI/180.0;

	/*
	strcpy(ulink[ULINK_ID_6].name, "LINK5");  
	ulink[ULINK_ID_6].sister = 0;
	ulink[ULINK_ID_6].child = 0;
	ulink[ULINK_ID_6].b[0] = 0.0; 
	ulink[ULINK_ID_6].b[1] = 0.0; 
	ulink[ULINK_ID_6].b[2] = 67.25; 
	*/

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


void RobotData::InverseKinematics(double p[3], double pitch, double yaw){
	
	double pitch_R[3][3] = {0.0, 0.0, 0.0};
	double yaw_R[3][3] = {0.0, 0.0, 0.0};
	double lambda = 0.1;
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

	RCLCPP_INFO(this->get_logger(),"init");
	for(int i = 0 ; i < 3 ; i++){
		RCLCPP_INFO(this->get_logger(),"ulink[ULINK_ID_6].p[%d] = %f",i,ulink[ULINK_ID_6].p[i]);
	}
	
	//int n = 0;
	for(int n = 0 ; n < 100 ; n++){
	//for(;;){

		CalcJacobian(Jacobian);

		CalcVWerr(err);

		if(matrix->Norm51(err) < 1E-06)
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

		RCLCPP_INFO(this->get_logger(),"n = %d",n);
		RCLCPP_INFO(this->get_logger(),"loop");
		for(int i = 0 ; i < 3 ; i++){
			RCLCPP_INFO(this->get_logger(),"ulink[ULINK_ID_6].p[%d] = %f",i,ulink[ULINK_ID_6].p[i]);
		}

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


void RobotData::TestPublish(){

	//RCLCPP_INFO(this->getlogger(),"*** position ***");
	
	/*
	for(int i = 1 ; i < ULINK_NUM ; i++){
		RCLCPP_INFO(this->get_logger(),"ulink[%d].mother = %d",i,ulink[i].mother);
	}
	*/

	
	
	double target_p[3] = {100.0, 100.0, 150.0};
	double pitch = 180.0 * M_PI / 180.0;
	double yaw = 0.0 * M_PI / 180.0;

	InverseKinematics(target_p, pitch, yaw);
	
	
	
	
//	ForwardKinematics(ULINK_ID_1);
	
/*
	for(int j = 0 ; j < ULINK_INDEX_NUM ; j++){
		RCLCPP_INFO(this->get_logger(),"ulink[%d].name = %s",j,ulink[j].name);
		for(int i = 0 ; i < 3 ; i++){
			RCLCPP_INFO(this->get_logger(),"ulink[%d].p[%d] = %f",j,i,ulink[j].p[i]);
		}
	}
	*/

	/*
	for(int i = 0 ; i < 3 ; i++){
		RCLCPP_INFO(this->get_logger(),"ulink[ULINK_ID_6].p[%d] = %f",i,ulink[ULINK_ID_6].p[i]);
	}
	*/
	
	
	
	
}
