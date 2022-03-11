

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_data/matrix.hpp"

#include <stdio.h>
#include <stdlib.h>

#include <math.h>


using std::placeholders::_1;



typedef struct ULINK{

	/* [3]     3*1 Matrix */
	/* [3][3]  3*3 Matrix */

	char name[20];
	//double mass; // [kg]
	int sister;
	int child;
	int mother;
	double p[3]; 	//position (world) [mm] 
	double R[3][3]; //R	(world)
	float q; // [rad]
	double a[3];    	 
	double b[3];    //[mm]	

}ULINK;	



enum ulink_index{

	ULINK_ID_1 = 1,
	ULINK_ID_2,
	ULINK_ID_3,
	ULINK_ID_4,
	ULINK_ID_5,
	ULINK_NUM
};


/*
enum vector{

	X,
	Y,
	Z
};
*/

class RobotData : public rclcpp::Node{

private:
	ULINK ulink[ULINK_NUM];

public:
	Matrix *matrix;

	double link_l[ULINK_NUM];
	

	void Initialize();
	void FindMother(int j);

	void ForwardKinematics(int j);
	void Rodrigues(double Rod[3][3], double w[3], double dt);

	void Output();

	RobotData() : Node("RobotData"){
		matrix = new Matrix();
	}

	~RobotData(){
		delete(matrix);
	}

};
