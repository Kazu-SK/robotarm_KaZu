

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_data/matrix.hpp"

#include "kinematics_service/srv/inv_kinematics.hpp"

#include <stdio.h>
#include <stdlib.h>

#include <math.h>


using namespace std::placeholders;



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
	double q; // [rad]
	double a[3];    	 
	double b[3];    //[mm]	

}ULINK;	



enum ulink_index{

	ULINK_ID_T,
	ULINK_ID_1,
	/* link information*/
	ULINK_ID_2,
	ULINK_ID_3,
	ULINK_ID_4,
	ULINK_ID_5,
	ULINK_ID_6,
	/* index num */
	ULINK_INDEX_NUM
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
	ULINK ulink[7];

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Service<kinematics_service::srv::InvKinematics>::SharedPtr service;

public:
	Matrix *matrix;

	void Initialize();
	void FindMother(int j);

	void ForwardKinematics(int j);
	void Rodrigues(double Rod[3][3], double w[3], double dt);

	void InverseKinematicsNum(double p[3], double pitch, double yaw);

	void InverseKinematicsAna(double p[3], double pitch, double yaw);
	

	void CalcJacobian(double J[5][5]);
	void CalcVWerr(double e[5]);
	void Rot2Omega(double w[3], double R[3][3]);

	void InvKinemaService(const std::shared_ptr<kinematics_service::srv::InvKinematics::Request> request,std::shared_ptr<kinematics_service::srv::InvKinematics::Response> response);


	RobotData() : Node("RobotData"){
		matrix = new Matrix();

		Initialize();

		service = create_service<kinematics_service::srv::InvKinematics>("inverse_kinematics", std::bind(&RobotData::InvKinemaService, this, _1, _2));
	}

	~RobotData(){
		delete(matrix);
	}


};
