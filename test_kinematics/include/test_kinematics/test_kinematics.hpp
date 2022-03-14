
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


class TestKinematics : public rclcpp::Node{


private:

public:
	TestKinematics()
	: Node("test_kinematics")
	{
		TestKinematicsInitialize();

	}

	void TestKinematicsInitialize();
};
