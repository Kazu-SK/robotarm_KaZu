
#include "robot_data/matrix.hpp"


void Matrix::Initialize(){



}

void Matrix::SumMatrix31(double a[3], double b[3], double c[3]){

	for(int i = 0 ; i < 3 ; i++){
		a[i] = b[i] + c[i];
	}
}

void Matrix::MultiMatrix31(double a[3], double b[3][3], double c[3]){

	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 3 ; j++){
			a[i] +=  b[i][j] * c[j];
		}
	}

}

void Matrix::SubstituteMatrix31(double a[3], double b[3]){

	for(int i = 0 ; i < 3 ; i++){
		a[i] = b[i];
	}
}


void Matrix::EyeMatrix33(double a[3][3]){

	double eye[3][3] = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1}
	};


	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 3 ; j++){
			a[i][j] = eye[i][j];
		}
	}

}

void Matrix::InitMatrix33(double a[3][3]){


	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 3 ; j++){
			a[i][j] = 0.0;
		}
	}
}



void Matrix::MultiMatrix33(double a[3][3], double b[3][3], double c[3][3]){


	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 3 ; j++){
			for(int k = 0 ; k < 3 ; k++){
				a[i][j] += b[i][k] * c[k][j];
			}
		}
	}
}


void Matrix::Roll(double a[3][3], float b){

	double roll[3][3] = {

		{1.0,	0.0, 	0.0},
		{0.0,	cos(b),	-sin(b)},
		{0.0,	sin(b),	cos(b)}
	};


	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 3 ; j++){
			a[i][j] = roll[i][j];
		}
	}
}


void Matrix::Pitch(double a[3][3], float b){

	double pitch[3][3] = {

		{cos(b),	0.0,	sin(b)},
		{0.0,		1.0,	0.0},
		{-sin(b),	0.0,	cos(b)}
	};


	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 3 ; j++){
			a[i][j] = pitch[i][j];
		}
	}
}


void Matrix::Yaw(double a[3][3], float b){

	double yaw[3][3] = {

		{cos(b),	-sin(b),	0.0},
		{sin(b),	cos(b),		0.0},
		{0.0,		0.0,		1.0}
	};


	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 3 ; j++){
			a[i][j] = yaw[i][j];
		}
	}

}






