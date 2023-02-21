#pragma once

#include <iostream>
#include <string>
#include <functional>
#include <Eigen/Dense>

using namespace std;

extern "C" {
#include "extApi.h"
}

const std::string JOINT_HANDLE_PREFIX{ "panda_joint" };
const std::string GRIPPER_HANDLE_PREFIX{ "panda_finger_joint" };
const std::string SENSOR_HANDLE_PREFIX{ "Force_sensor" };

class VRepBridge
{
public:
	enum ControlMode { CTRL_POSITION, CTRL_VELOCITY, CTRL_TORQUE };

	VRepBridge(ControlMode mode = CTRL_POSITION);
	~VRepBridge();
	
	bool simConnectionCheck();
	void simLoop();

	void write();
	void read();

	void setDesiredPosition(const Eigen::Matrix<double, DOF, 1> & desired_q);
	void setGripperDesiredPosition(const Eigen::Matrix<double, 2, 1> & desired_gq);
	void setDesiredTorque(const Eigen::Matrix<double, DOF, 1> & desired_torque);
	const Eigen::Matrix<double, DOF, 1> & getPosition();
	const Eigen::Matrix<double, DOF, 1> & getVelocity();
	const Eigen::Matrix<double, 2, 1> & getGripperPosition();
	const Eigen::Matrix<double, 6, 1> & getFTData();

	const size_t getTick() { return tick_; }

private:
	Eigen::Matrix<double, DOF, 1> current_q_;
	Eigen::Matrix<double, DOF, 1> current_q_dot_;
	Eigen::Matrix<double, DOF, 1> desired_q_;
	Eigen::Matrix<double, DOF, 1> desired_torque_;
	Eigen::Matrix<double, 2, 1> current_gq_;
	Eigen::Matrix<double, 2, 1> desired_gq_;
	Eigen::Matrix<double, 6, 1> current_ft_;

	simxInt clientID_;
	simxInt motorHandle_[DOF];	/// < Depends on simulation envrionment
	simxInt gripperHandle_[2];
	simxInt sensorHandle_;
	simxInt objectHandle_;

	size_t tick_{ 0 };
	
	ControlMode control_mode_;

	void simxErrorCheck(simxInt error);
	void simInit();
	void getHandle(); 	/// < Depends on simulation envrionment
};
