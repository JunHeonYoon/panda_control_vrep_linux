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
	Vector2d gq_init_; // for gripper
	Vector2d gqdot_init_; // for gripper
	Vector2d gforce_init_; // for gripper

	// Current state
	Vector7d q_;
	Vector7d qdot_;
	Vector7d qddot_;
	Vector7d torque_;
	Vector7d q_error_sum_;
	Vector2d gq_;
	Vector2d gqdot_;
	Vector2d gforce_;
	Vector6d tip_ft_; // FT sensor data (force, torque)

	// Control value (position controlled)
	Vector7d q_desired_; // Control value
	Vector7d torque_desired_;
	Vector2d gq_desired_;
	Vector2d gqdot_desired_;
	Vector2d gforce_desired_;

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
	unsigned long control_start_tick_;

	std::string control_mode_;
	bool is_mode_changed_;

	// for robot model construction
	Math::Vector3d com_position_[DOF];
	Vector3d joint_posision_[DOF];

	shared_ptr<Model> model_;
	unsigned int body_id_[DOF];
	Body body_[DOF];
	Joint joint_[DOF];

	std::vector<MatrixXd> traj_; // [x, y, z, x_dot, y_dot, z_dot]

private:
	MatrixXd jacobianFromqd(int mode); // mode=0 : end-effector, 1:COM of link
	void printState();
	void moveJointPosition(const Vector7d &target_position, double duration);
	void moveJointPositionTorque(const Vector7d &target_position, double duration);
	void moveGripperPosition(const Vector2d &target_position, double duration);
	void setGripperForce(const Vector2d &target_force, double duration);
	void setGripperVelocity(const Vector2d &target_velocity, double duration);
	void moveGripperPositionForce(const Vector2d &target_position, double duration);
	void simpleJacobianControl(const Vector12d &target_x, double duration);
	void feedbackJacobianControl(const Vector12d & target_x, double duration);
	void CLIK(const Vector12d & target_x, double duration);
	void CLIKwTraj(const int traj_type);

public:
	void readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque);
	void readData(const Vector7d &position, const Vector7d &velocity);
	// void readData(const Vector7d &position, const Vector7d &velocity, const Vector2d &gripper_position, const Vector6d &tip_ft);
	void readData(const Vector7d &position, const Vector7d &velocity, 
				  const Vector2d &gripper_position, const Vector2d &gripper_velocity, const Vector2d &gripper_force,
				  const Vector6d &tip_ft);

	const Vector7d & getDesiredPosition();
	const Vector7d & getDesiredTorque();
	const Vector2d & getDesiredGripperPosition();
	const Vector2d & getDesiredGripperVelocity();
	const Vector2d & getDesiredGripperForce();
	const Vector6d & getFTSensorData();

public:
		ArmController(double hz) :
		tick_(0), play_time_(0.0), hz_(hz), control_mode_("none"), is_mode_changed_(false)
	{
			initDimension(); initModel(); initFile(); readTrajFile();
	}


    void setMode(const std::string & mode);
    void initDimension();
    void initModel();
	void initFile();
	void readTrajFile();
    void initPosition();
    void compute();
private:
	ofstream debug_file_;
	constexpr static int NUM_HW_PLOT{20};
	ofstream hw_plot_files_[NUM_HW_PLOT];
	const string hw_plot_file_names_[NUM_HW_PLOT]
	{"simple", "feedback", "clik", "reference", "CIRCLE", "SQUARE", "EIGHT"};

	constexpr static int NUM_TRAJ_FILE{3};
	const string traj_file_names_[NUM_TRAJ_FILE]
	{"circle", "square", "eight"}; 

	void record(int file_number, double duration);
	void record(int file_number, double duration, const stringstream & ss);
	void recordFT(int file_number, double duration);
};
