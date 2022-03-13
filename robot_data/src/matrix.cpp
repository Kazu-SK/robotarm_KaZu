
#include "robot_data/matrix.hpp"


void Matrix::Initialize(){



}


/* 3*1 Matrix */

void Matrix::SumMatrix31(double a[3], double b[3], double c[3]){

	for(int i = 0 ; i < 3 ; i++){
		a[i] = b[i] + c[i];
	}
}

void Matrix::MultiMatrix31(double a[3], double b[3][3], double c[3]){

	for(int i = 0 ; i < 3 ; i++){

		a[i] = 0.0;

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


void Matrix::CrossMatrix31(double a[3], double b[3], double c[3]){

	a[0] = b[1] * c[2] - b[2] * c[1];
	a[1] = b[2] * c[0] - b[0] * c[2];
	a[2] = b[0] * c[1] - b[1] * c[0];
}


/* 3*3 Matrix */
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


	InitMatrix33(a);

	for(int i = 0 ; i < 3 ; i++){
		for(int j = 0 ; j < 3 ; j++){
			for(int k = 0 ; k < 3 ; k++){
				a[i][j] += b[i][k] * c[k][j];
			}
		}
	}
}

void Matrix::InverseMatrix33(double a[3][3], double b[3][3]){

	double buf;
	int i,j,k;
	int n=3;
	double b_buff[3][3];

	for(int t = 0 ; t < n ; t++){
		for(int u = 0 ; u < n ; u++){
			b_buff[t][u] = b[t][u];
		}
	}
	 
	for(i = 0 ; i < n; i++){
		 for(j = 0 ; j < n ; j++){
			a[i][j]=(i==j)?1.0:0.0;
		 }
	}

	for(i = 0 ; i < n ; i++){
		 buf = 1 / b[i][i];

		for(j = 0 ; j < n ; j++){
			b[i][j] *= buf;
			a[i][j] *= buf;
		}

		for(j = 0 ; j < n; j++){
			if(i != j){
				buf = b[j][i];
				for(k = 0 ; k < n ; k++){
					b[j][k] -= b[i][k] * buf;
					a[j][k] -= a[i][k] * buf;
				}
			}
		}
	}

	for(int t = 0 ; t < n ; t++){
		for(int u = 0 ; u < n ; u++){
			b[t][u] = b_buff[t][u];
		}
	}
}


double Matrix::TraceMatrix33(double a[3][3]){

	double ans = 0.0;

	for(int i = 0 ; i < 3 ; i++){

		ans += a[i][i];
	}

	return ans;
}

/* 5*1 Matrix */
void Matrix::MultiMatrix51(double a[5], double b[5][5], double c[5]){

	for(int i = 0 ; i < 5 ; i++){

		a[i] = 0.0;

		for(int j = 0 ; j < 5 ; j++){
			a[i] +=  b[i][j] * c[j];
		}
	}

}

/* 5*5 Matrix */

void Matrix::InitMatrix55(double a[5][5]){

	for(int i = 0 ; i < 5 ; i++){
		for(int j = 0 ; j < 5 ; j++){
			a[i][j] = 0.0;
		}
	}

}

void Matrix::InverseMatrix55(double a[5][5], double b[5][5]){

	double buf;
	int i,j,k;
	int n=5;
	double b_buff[5][5];


	InitMatrix55(b_buff);

	for(int t = 0 ; t < n ; t++){
		for(int u = 0 ; u < n ; u++){
			b_buff[t][u] = b[t][u];
		}
	}
	 
	for(i = 0 ; i < n; i++){
		 for(j = 0 ; j < n ; j++){
			a[i][j]=(i==j)?1.0:0.0;
		 }
	}

	for(i = 0 ; i < n ; i++){
		buf = 1 / b[i][i];

		for(j = 0 ; j < n ; j++){
			b[i][j] *= buf;
			a[i][j] *= buf;
		}

		for(j = 0 ; j < n; j++){
			if(i != j){
				buf = b[j][i];
				for(k = 0 ; k < n ; k++){
					b[j][k] -= b[i][k] * buf;
					a[j][k] -= a[i][k] * buf;
				}
			}
		}

	}

	for(int t = 0 ; t < n ; t++){
		for(int u = 0 ; u < n ; u++){
			b[t][u] = b_buff[t][u];
		}
	}
}

void Matrix::MultiMatrix55(double a[5][5], double b[5][5], double c[5][5]){

	InitMatrix55(a);

	for(int i = 0 ; i < 5 ; i++){
		for(int j = 0 ; j < 5 ; j++){
			for(int k = 0 ; k < 5 ; k++){
				a[i][j] += b[i][k] * c[k][j];
			}
		}
	}

	/*
	for(int t = 0 ; t < 5 ; t++){
		for(int u = 0 ; u < 5 ; u++){
			cout<<a[t][u]<<" ";
		}
		cout<<endl;
	}
	*/
}

/* Rotation Matrix*/
void Matrix::Roll(double a[3][3], double b){

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


void Matrix::Pitch(double a[3][3], double b){

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


void Matrix::Yaw(double a[3][3], double b){

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






