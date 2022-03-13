
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <iostream>

using namespace std;


class Matrix{

private:

public:
	void Initialize();

	/* 3*1 Matrix */
	double Norm31(double a[3]){ return sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]); };
	void SumMatrix31(double a[3], double b[3], double c[3]);
	void MultiMatrix31(double a[3], double b[3][3], double c[3]);
	void SubstituteMatrix31(double a[3], double b[3]);

	void CrossMatrix31(double a[3], double b[3], double c[3]);

	/* 3*3 Matrix */
	void EyeMatrix33(double a[3][3]);
	void InitMatrix33(double a[3][3]);
	void MultiMatrix33(double a[3][3], double b[3][3], double c[3][3]);
	void InverseMatrix33(double a[3][3], double b[3][3]);

	/* 5*5 Matrix */
	void InitMatrix55(double a[5][5]);
	void InverseMatrix55(double a[5][5], double b[5][5]);
	void MultiMatrix55(double a[5][5], double b[5][5], double c[5][5]);

	/* Rotation Matrix*/
	void Roll(double a[3][3], double b);
	void Pitch(double a[3][3], double b);
	void Yaw(double a[3][3], double b);



};
