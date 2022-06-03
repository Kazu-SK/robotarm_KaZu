

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_data/matrix.hpp"

#include "kinematics_service/srv/inv_kinematics.hpp"
//#include "vision_service/srv/coordinate_conversion.hpp"
#include "vision_interfaces/msg/image_coordinate.hpp"
#include "vision_interfaces/msg/world_coordinate.hpp"

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
	ULINK_ID_CAMERA,
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

class RobotData{// : public rclcpp::Node{

private:
	ULINK ulink[7];

	rclcpp::TimerBase::SharedPtr timer_;

	vision_interfaces::msg::ImageCoordinate image_coordinate_msg;
	vision_interfaces::msg::WorldCoordinate world_coordinate_msg;

	double world_coordinate[3];


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

	void CoordinateConversion(const vision_interfaces::msg::ImageCoordinate::SharedPtr msg);


	void getWorldCoordinate(double w_coordinate[]){
		
		for(int i = 0 ; i < 3 ; i++){
			w_coordinate[i] = world_coordinate[i];
		}
	};



	static const double FX;
	static const double FY;
	static const double CX;
	static const double CY;

	static const double PIXEL_SIZE; //[mm/pixel]
	static const double IMAGE_AREA_X;
	static const double IMAGE_AREA_Y;
	static const int REVOLUSION_H;
	static const int REVOLUSION_V;

	static const double OBJECT_HEIGHT; //[mm] 

	RobotData(){// : Node("RobotData"){

		matrix = new Matrix();

		Initialize();

		for(int i = 0 ; i < 3 ; i++){
			world_coordinate[i] = 0.0;
		}
	}

	~RobotData(){
		delete(matrix);
	}


};
