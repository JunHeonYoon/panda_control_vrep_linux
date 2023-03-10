#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <rbdl/rbdl.h>
#include <memory>
#include <fstream>
#include "math_type_define.h"

#define EYE(X) Matrix<double, X, X>::Identity()

using namespace RigidBodyDynamics;
using namespace std;
using namespace Eigen;

class ArmController
{
	size_t dof_;

	// Initial state
	Vector7d q_init_;
	Vector7d qdot_init_;

	// Current state
	Vector7d q_;
	Vector7d qdot_;
	Vector7d qddot_;
	Vector7d torque_;
	Vector7d q_error_sum_;

	// Control value (position controlled)
	Vector7d q_desired_; // Control value
	Vector7d torque_desired_;

	// Task space
	Vector3d x_init_;
	Vector3d x_2_init_;
	Vector3d x_;
	Matrix3d rotation_;
	Matrix3d rotation_init_;
	Vector3d phi_;
	Vector6d x_dot_;   // 6D (linear + angular)
	Vector6d x_2_dot_; // 6D (linear + angular)
	Vector6d x_error_;

	// Dynamics
	Vector7d g_; // Gravity torque
	Matrix7d m_; // Mass matrix
	Matrix7d m_inverse_; // Inverse of mass matrix

	// For controller
	Matrix<double, 3, 7> j_v_;	// Linear velocity Jacobian matrix
	Matrix<double, 3, 7> j_w_;	// Angular veolicty Jacobain matrix
	Matrix<double, 6, 7> j_;	// Full basic Jacobian matrix
	Matrix<double, 7, 6> j_inverse_;	// Jacobain inverse storage 

	VectorXd q_temp_;	// For RBDL 
	VectorXd qdot_temp_;
	VectorXd qddot_temp_;
	MatrixXd j_temp_;	// For RBDL 
	MatrixXd m_temp_;
	VectorXd g_temp_;   // For RBDL 

	Vector7d q_cubic_;
	Vector7d q_target_;

	Vector3d x_cubic_;
	Vector3d x_cubic_old_;
	Vector3d x_target_;


	Vector3d x_2_; //4번 링크 CoM 위치
	Matrix<double, 6, 7> j_2_; //4번 링크 CoM의 jacobian matrix
	MatrixXd j_temp_2_; //각각 x_, j_, j_temp_2 변수 선언 밑에 작성


	unsigned long tick_;
	double play_time_;
	double hz_;
	double control_start_time_;

	std::string control_mode_;
	bool is_mode_changed_;

	// for robot model construction
	Math::Vector3d com_position_[DOF];
	Vector3d joint_posision_[DOF];

	shared_ptr<Model> model_;
	unsigned int body_id_[DOF];
	Body body_[DOF];
	Joint joint_[DOF];

private:
	MatrixXd jacobianFromqd(int mode); // mode=0 : end-effector, 1:COM of link
	void printState();
	void moveJointPosition(const Vector7d &target_position, double duration);
	void moveJointPositionTorque(const Vector7d &target_position, double duration);
	void simpleJacobianControl(const Vector12d &target_x, double duration);
	void feedbackJacobianControl(const Vector12d & target_x, double duration);
	void CLIK(const Vector12d & target_x, double duration);
	void hw_2_1(const Vector12d & target_x, double duration);
	void hw_2_2(const Vector12d & target_x, double duration);
	void hw_2_3(const Vector12d & target_x, double duration);
	void hw_3_1(const Vector6d & target_x, double duration);
	void hw_3_2(const Vector6d & target_x, double duration);
	void hw_3_3(const Vector6d & target_x, double duration, bool isStep);
	void hw_4_1(const Vector7d & target_q, double duration);
	void hw_4_2(const Vector7d & target_q, double duration, bool isStep);
	void hw_4_3(const Vector7d & target_q, double duration, bool isStep);
	void hw_5_1(const Vector12d & target_x, double duration, bool isStep);
	void hw_5_2(const Vector12d & target_x, double duration);
	void hw_7(const Vector12d & target_x, double duration);


public:
	void readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque);
	void readData(const Vector7d &position, const Vector7d &velocity);
	const Vector7d & getDesiredPosition();
	const Vector7d & getDesiredTorque();

public:
		ArmController(double hz) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
	{
			initDimension(); initModel(); initFile();
	}


    void setMode(const std::string & mode);
    void initDimension();
    void initModel();
	void initFile();
    void initPosition();
    void compute();
private:
	ofstream debug_file_;
	constexpr static int NUM_HW_PLOT{20};
	ofstream hw_plot_files_[NUM_HW_PLOT];
	const string hw_plot_file_names_[NUM_HW_PLOT]
	{"simple", "feedback", "clik", "reference", "hw_2_1", "hw_2_2", "hw_2_3", "hw_3_1", "hw_3_2", "hw_3_3_1", "hw_3_3_2"
	, "hw_4_1", "hw_4_2", "hw_4_3_1", "hw_4_3_2", "hw_5_1_1", "hw_5_1_2", "hw_5_2", "hw_7"};

	void record(int file_number, double duration);
	void record(int file_number, double duration, const stringstream & ss);
	void recordHW2(int file_number, double duration, const Vector3d & x_desired);
	void recordHw3(int file_number, double duration, const Vector6d & x_desired);
	void recordHw4(int file_number, double duration, const Vector7d & q_desired);
	void recordHw5(int file_number, double duration, const Vector3d & x_desired, const Vector6d & xd_desired);
};
