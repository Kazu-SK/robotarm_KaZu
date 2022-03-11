
#include "robot_data/robot_data.hpp"


void RobotData::Initialize(){

	std::cout<<"robotdata_initialize"<<std::endl;

	//double UX[3] = {1.0, 0.0, 0.0};
	double UY[3] = {0.0, 1.0, 0.0};
	double UZ[3] = {0.0, 0.0, 1.0}; 

	strcpy(ulink[ULINK_ID_1].name, "LINK1");  
	ulink[ULINK_ID_1].sister = 0;
	ulink[ULINK_ID_1].child = 2;  
	ulink[ULINK_ID_1].b[0] = 0.0; 
	ulink[ULINK_ID_1].b[1] = 0.0; 
	ulink[ULINK_ID_1].b[2] = 220.0; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_1].a, UZ);
	ulink[ULINK_ID_1].q = 0.0;
	matrix->Yaw(ulink[ULINK_ID_1].R,ulink[ULINK_ID_1].q); 

	strcpy(ulink[ULINK_ID_2].name, "LINK2");  
	ulink[ULINK_ID_2].sister = 0;
	ulink[ULINK_ID_2].child = 3;
	ulink[ULINK_ID_2].b[0] = 0.0; 
	ulink[ULINK_ID_2].b[1] = 0.0; 
	ulink[ULINK_ID_2].b[2] = 95.0; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_2].a, UY);
	ulink[ULINK_ID_2].q = 0.0;

	strcpy(ulink[ULINK_ID_3].name, "LINK3");  
	ulink[ULINK_ID_3].sister = 0;
	ulink[ULINK_ID_3].child = 4;
	ulink[ULINK_ID_3].b[0] = 0.0; 
	ulink[ULINK_ID_3].b[1] = 0.0; 
	ulink[ULINK_ID_3].b[2] = 95.0; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_3].a, UY);
	ulink[ULINK_ID_3].q = 0.0;

	strcpy(ulink[ULINK_ID_4].name, "LINK4");  
	ulink[ULINK_ID_4].sister = 0;
	ulink[ULINK_ID_4].child = 5;
	ulink[ULINK_ID_4].b[0] = 0.0; 
	ulink[ULINK_ID_4].b[1] = 0.0; 
	ulink[ULINK_ID_4].b[2] = 77.75; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_4].a, UY);
	ulink[ULINK_ID_4].q = 0.0;

	strcpy(ulink[ULINK_ID_4].name, "LINK5");  
	ulink[ULINK_ID_5].sister = 0;
	ulink[ULINK_ID_5].child = 0;
	ulink[ULINK_ID_5].b[0] = 0.0; 
	ulink[ULINK_ID_5].b[1] = 0.0; 
	ulink[ULINK_ID_5].b[2] = 67.25; 
	matrix->SubstituteMatrix31(ulink[ULINK_ID_5].a, UZ);
	ulink[ULINK_ID_5].q = 0.0;

	FindMother(1);
}


void RobotData::FindMother(int j){

	if(j != 0){

		if(j == 1){

			ulink[j].mother = 0;
		}

		if(ulink[j].child != 0){
			ulink[ulink[j].child].mother = j;
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
		matrix->SumMatrix31(ulink[j].p, val, ulink[mom].p);

		Rodrigues(Rod, ulink[j].a, ulink[j].q);
		matrix->MultiMatrix33(ulink[j].R, ulink[mom].R, Rod);
	}

	ForwardKinematics(ulink[j].sister);
	ForwardKinematics(ulink[j].child);
}	


void RobotData::Rodrigues(double Rod[3][3], double w[3], double dt){

	double th = 0.0;
	double wn[3] = {0, 0, 0};
	double multi_wedge[3][3] = {{0, 0, 0},{0, 0, 0},{0, 0, 0}};
	double eye[3][3] = {{1, 0, 0},{0, 1, 0},{0, 0, 1}};

	

	th = matrix->Norm31(w) * dt;

	wn[0] = w[0]/matrix->Norm31(w);
	wn[1] = w[1]/matrix->Norm31(w);
	wn[2] = w[2]/matrix->Norm31(w);

	double w_wedge[3][3] = {
		{0	,wn[2] 	,wn[1]},
		{wn[2]	,0	,wn[0]},
		{wn[1]	,wn[0]	,0    }
	};

	matrix->MultiMatrix33(multi_wedge, w_wedge, w_wedge);

	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 3 ; j++){

			Rod[i][j] = eye[i][j] + w_wedge[i][j] * sin(th) + multi_wedge[i][j] * (1 - cos(th)); 	
		}
	}
	
}



void RobotData::Output(){

	std::cout<<"p[0] = "<<ulink[ULINK_ID_5].p[0]<<std::endl;
	std::cout<<"p[1] = "<<ulink[ULINK_ID_5].p[1]<<std::endl;
	std::cout<<"p[2] = "<<ulink[ULINK_ID_5].p[2]<<std::endl;

}
