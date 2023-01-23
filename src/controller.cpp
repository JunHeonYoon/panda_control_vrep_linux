#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>


MatrixXd ArmController::jacobianFromqd(int mode)
{
	Vector3d x_from_q_desired;
	MatrixXd j_temp;
	j_temp.resize(6, DOF);
	j_temp.setZero();

	Matrix<double, 6, 7> j_from_q_desired;
	if(mode == 0) 
	{
		x_from_q_desired = CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
		CalcPointJacobian6D(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp, false);
	}
	else if(mode == 1)
	{ 
		x_from_q_desired = CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 4], com_position_[DOF - 4], false);
		CalcPointJacobian6D(*model_, q_desired_, body_id_[DOF - 4], com_position_[DOF - 4], j_temp, false);
	}

	for(int i=0;i<2;i++)
	{
		j_from_q_desired.block<3, DOF>(i * 3, 0) = j_temp.block<3, DOF>(3 - i * 3, 0);
	}
	
	return j_from_q_desired;
	
}

void ArmController::compute()
{
	// Kinematics and dynamics calculation ------------------------------
	q_temp_ = q_;
	qdot_temp_ = qdot_;

	RigidBodyDynamics::UpdateKinematicsCustom(*model_, &q_temp_, &qdot_temp_, NULL);
	x_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], true);
	x_2_ = CalcBodyToBaseCoordinates(*model_, q_, body_id_[DOF - 4], com_position_[DOF - 4], true);
	
	rotation_ = CalcBodyWorldOrientation(*model_, q_, body_id_[DOF - 1], true).transpose();
	Matrix3d body_to_ee_rotation;
	body_to_ee_rotation.setIdentity();
	body_to_ee_rotation(1, 1) = -1;
	body_to_ee_rotation(2, 2) = -1;
	rotation_ = rotation_ * body_to_ee_rotation;
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 1], com_position_[DOF - 1], j_temp_, true);
	CalcPointJacobian6D(*model_, q_, body_id_[DOF - 4], com_position_[DOF - 4], j_temp_2_, true);


	NonlinearEffects(*model_, q_, Vector7d::Zero(), g_temp_);
	CompositeRigidBodyAlgorithm(*model_, q_, m_temp_, true);

	g_ = g_temp_;
	m_ = m_temp_;
	m_inverse_ = m_.inverse();

	for (int i = 0; i<2; i++)
	{
		j_.block<3, DOF>(i * 3, 0) = j_temp_.block<3, DOF>(3 - i * 3, 0);
		j_2_.block<3, DOF>(i * 3, 0) = j_temp_2_.block<3, DOF>(3 - i * 3, 0);
	}
	// -----------------------------------------------------
	
	
	// ---------------------------------
	//
	// q_		: joint position
	// qdot_	: joint velocity
	// x_		: end-effector position 
	// j_		: end-effector basic jacobian
	// m_		: mass matrix
	//
	//-------------------------------------------------------------------
	
	j_v_ = j_.block < 3, DOF>(0, 0);

	x_dot_ = j_ * qdot_;
	x_2_dot_ = j_2_ * qdot_;
		
	
	
	if (is_mode_changed_)
	{
		is_mode_changed_ = false;

		control_start_time_ = play_time_;

		q_init_ = q_;
		qdot_init_ = qdot_;
		q_error_sum_.setZero();

		x_init_ = x_;
		x_2_init_ = x_2_;
		x_cubic_old_ = x_;
		rotation_init_ = rotation_;
	}

	if (control_mode_ == "joint_ctrl_home")
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, M_PI / 4;
		// moveJointPositionTorque(target_position, 1.0);
		moveJointPosition(target_position, 1.0);
	}
	else if(control_mode_ == "joint_ctrl_init")
	{
		Vector7d target_position;
		target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, M_PI / 2, 0;
		// target_position << 0.0, 0.0, 0.0, -M_PI / 6., 0.0, M_PI / 2, 0;
		// target_position << 0.0, -M_PI / 3, 0.0, -M_PI / 2., 0.0, M_PI / 6, 0;
		// moveJointPositionTorque(target_position, 1.0);     
		moveJointPosition(target_position, 1.0);                
	}
	else if (control_mode_ == "simple_jacobian")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			0, -1, 0,
			-1, 0, 0,
			0, 0, -1;
		simpleJacobianControl(target_x, 2.0);
	}
	else if (control_mode_ == "feedback_jacobian")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			0, -1, 0,
			-1, 0, 0,
			0, 0, -1;
		feedbackJacobianControl(target_x, 2.0);
	}
	else if (control_mode_ == "CLIK")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			0, -1, 0,
			-1, 0, 0,
			0, 0, -1;
		CLIK(target_x, 2.0);
	}
	else if (control_mode_ == "hw_2_1")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			1, 0, 0,
			0, -1, 0,
			0, 0, -1;
		hw_2_1(target_x, 4.0);
	}
	else if (control_mode_ == "hw_2_2")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			1, 0, 0,
			0, -1, 0,
			0, 0, -1;
		hw_2_2(target_x, 4.0);
	}
	else if (control_mode_ == "hw_2_3")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
			1, 0, 0,
			0, -1, 0,
			0, 0, -1;
		hw_2_3(target_x, 4.0);
	}
	else if (control_mode_ == "hw_3_1")
	{
		Vector6d target_x;
		target_x << 0.25,  0.28, 0.65,
					0.00, -0.15, 0.60;
		hw_3_1(target_x, 4.0);
	}
	else if (control_mode_ == "hw_3_2")
	{
		Vector6d target_x;
		target_x << 0.25,  0.28, 0.65,
					0.00, -0.15, 0.60;
		hw_3_2(target_x, 4.0);
	}
	else if (control_mode_ == "hw_3_3")
	{
		Vector6d target_x;
		target_x << 0.25,  0.28, 0.65,
					0.00, -0.15, 0.60;
		hw_3_3(target_x, 4.0, false); // true : hw_3_3_1, false : hw_3_3_2
	}
	else if (control_mode_ == "hw_4_1")
	{
		Vector7d target_q;
		target_q << 0.0, 0.0, 0.0, -M_PI*(25./180.), 0.0, M_PI / 2, 0; 
		hw_4_1(target_q, 4.0);
	}
	else if (control_mode_ == "hw_4_2")
	{
		Vector7d target_q;
		// target_q << 0.0, 0.0, 0.0, -M_PI*(25./180.), 0.0, M_PI / 2, 0;
		target_q << 0.0, 0.0, 0.0, -M_PI/3., 0.0, M_PI / 2, 0; 
		hw_4_2(target_q, 4.0, false);
	}
	else if (control_mode_ == "hw_4_3")
	{
		Vector7d target_q;
		target_q << 0.0, 0.0, 0.0, -M_PI*(25./180.), 0.0, M_PI / 2, 0;
		// target_q << 0.0, 0.0, 0.0, -M_PI/3., 0.0, M_PI / 2, 0; 
		hw_4_3(target_q, 4.0, true);
	}
	else if (control_mode_ == "hw_5_1")
	{
		Vector12d target_x;
		target_x << x_init_, rotation_init_.col(0), rotation_init_.col(1), rotation_init_.col(2);
		// target_x(1) += 0.02;
		target_x(1) += 0.1; 
		hw_5_1(target_x, 4.0, false);
	}
	else if (control_mode_ == "hw_5_2")
	{
		Vector12d target_x;
		target_x << 0.3, -0.012, 0.52,
				rotation_init_.col(0), rotation_init_.col(1), rotation_init_.col(2);
		hw_5_2(target_x, 4.0);
	}
	else if (control_mode_ == "hw_7")
	{
		Vector12d target_x;
		target_x << 0.3, -0.012, 0.52,
				rotation_init_.col(0), rotation_init_.col(1), rotation_init_.col(2);
		hw_7(target_x, 8.0);
	}
	else
	{
		torque_desired_ = g_;
	}

	printState();

	tick_++;
	play_time_ = tick_ / hz_;	// second
}

void ArmController::record(int file_number, double duration)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] << x_.transpose() <<
			Map< Matrix<double, 1, 9> >(rotation_.data(), rotation_.size()) << x_cubic_.transpose() <<
			endl;
	}
}

void ArmController::record(int file_number, double duration, const stringstream & ss)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] << ss.str() << endl;
	}
}

void ArmController::recordHW2(int file_number, double duration, const Vector3d & x_desired)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] 
		<< x_.transpose()<< " " << x_desired.transpose() << " " 
		<< q_desired_.transpose()
		<< endl;
	}
}

void ArmController::recordHw3(int file_number, double duration, const Vector6d &x_desired)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] 
		<< x_.transpose()<< " " << x_2_.transpose()<< " "
		<< x_desired.transpose() <<  " "
		<< q_desired_.transpose()
		<< endl;
	}
}

void ArmController::recordHw4(int file_number, double duration, const Vector7d & q_desired)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] 
		<< q_desired.transpose() << " "
		<< q_.transpose() <<  " "
		<< torque_desired_.transpose()
		<< endl;
	}
	else if (play_time_ >= control_start_time_ + duration + 1.0)
	{
		cout << "Record end" << endl;
	}
}

void ArmController::recordHw5(int file_number, double duration, const Vector3d & x_desired, const Vector6d & xd_desired)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] 
		<< x_.transpose()<< " " 
		<< x_desired.transpose() <<  " "
		<< xd_desired.transpose() << " "
		<< x_dot_.transpose()
		<< endl;
	}
}


void ArmController::printState()
{
	// TODO: Modify this method to debug your code

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 50.)
	{
		DBG_CNT = 0;

		cout << "time     : " << std::fixed << std::setprecision(3) << play_time_ << endl;
		cout << "q now    :\t";
		cout << std::fixed << std::setprecision(3) << q_.transpose() << endl;
		cout << "q desired:\t";
		cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << endl;
		cout << "x        :\t";
		cout << x_.transpose() << endl;
		cout << "R        :\t" << endl;
		cout << std::fixed << std::setprecision(3) << rotation_ << endl;
		
		if(control_mode_ == "hw_3_1")
		{
			Matrix<double, 6, 7>j;
			j <<  j_.block <3, DOF>(0, 0),
			j_2_.block<3, DOF>(0, 0); 
			
			cout << "hw 3-1 jacobian:" << endl;
			cout << j << endl;

			Vector6d x;
			x << x_, x_2_;

			cout << "hw 3-1 x:\t" << x.transpose() << endl;
		}
		
	}
}

void ArmController::moveJointPosition(const Vector7d & target_position, double duration)
{
	Vector7d zero_vector;
	zero_vector.setZero();
	q_desired_ = DyrosMath::cubicVector<7>(play_time_,
		control_start_time_,
		control_start_time_ + duration, q_init_, target_position, zero_vector, zero_vector);
}

void ArmController::moveJointPositionTorque(const Vector7d &target_position, double duration)
{
	Matrix7d kp, kv;
	Vector7d q_cubic, qd_cubic;
	
	kp = Matrix7d::Identity() * 500.0;
	kv = Matrix7d::Identity() * 20.0;

	for (int i = 0; i < 7; i++)
	{
		qd_cubic(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_position(i), 0, 0);
		q_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_position(i), 0, 0);
	}


	torque_desired_ = m_ * (kp*(q_cubic - q_) + kv*(qd_cubic - qdot_)) + g_;
}

void ArmController::simpleJacobianControl(const Vector12d & target_x, double duration)
{
	Vector6d xd_desired;
	for (int i = 0; i < 3; i++)
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}
	Matrix3d rotation;

	for (int i = 0; i < 3; i++)
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i*3);
	}
	xd_desired.segment<3>(3) = DyrosMath::rotationCubicDot(play_time_, control_start_time_,
		control_start_time_ + duration, Vector3d::Zero(), Vector3d::Zero(), rotation_init_, rotation);

	// debug_file_ << xd_desired.transpose() << endl;
	// xd_desired.segment<3>(3).setZero();
	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * xd_desired;
	
	q_desired_ = q_desired_ + qd_desired / hz_;
	record(0, duration);
}

void ArmController::feedbackJacobianControl(const Vector12d & target_x, double duration)
{
	Vector6d delta_x_desired;
	Vector3d x_cubic;
	Matrix3d rotation;

	for (int i = 0; i < 3; i++)
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i * 3);
	}

	for (int i = 0; i < 3; i++)
	{
		x_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}

	Matrix3d rotation_cubic = DyrosMath::rotationCubic(play_time_, control_start_time_,
		control_start_time_ + duration, rotation_init_, rotation);
	delta_x_desired.segment<3>(0) = x_cubic - x_;
	delta_x_desired.segment<3>(3) = - 0.5 * DyrosMath::getPhi(rotation_, rotation_cubic) ;

	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * delta_x_desired;

	q_desired_ = q_ + qd_desired;

	stringstream ss;
	ss << x_cubic.transpose() <<
		Map< Matrix<double, 1, 9> >(rotation_cubic.data(), rotation_cubic.size());
	record(1, duration);
	record(3, duration, ss);
}


void ArmController::CLIK(const Vector12d & target_x, double duration)
{
	//------------------------------------------------------------------------
	// 먼저 주어진 data (최종 목표 pose, 도달 시간)을 통해 현재 시간에 맞는 desired pose와 
	// desired velocity를 구한다.
	// 여기서 초기 pose는 주어진 상태이고, 초기와 최종 velocity는 0으로 가정한다.

	Vector6d xd_desired, x_error;
	Vector3d x_cubic;

	for (int i = 0; i < 3; i++) // 현재 시간에 맞는 desired linear velocity를 구한다.
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}
	Matrix3d rotation;

	for (int i = 0; i < 3; i++) // 현재 시간에 맞는 desired angular velocity를 구한다.
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i * 3);
	}
	xd_desired.segment<3>(3) = DyrosMath::rotationCubicDot(play_time_, control_start_time_,
		control_start_time_ + duration, Vector3d::Zero(), Vector3d::Zero(), rotation_init_, rotation);

	for (int i = 0; i < 3; i++) // 현재 시간에 맞는 desired position을 구한다.
	{
		x_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}

	// 현재 시간에 맞는 desired ratation을 구한다.
	Matrix3d rotation_cubic = DyrosMath::rotationCubic(play_time_, control_start_time_,
		control_start_time_ + duration, rotation_init_, rotation);
	//---------------------------------------------------------------------------

	//---------------------------------------------------------------------------
	// 그 다음 계산한 값들을 기반으로 CLIK control을 진행한다.
	// 여기서 현재 jacobian과 end-effector의 pose는 실시간으로 받아온다.

	// Feedback control을 위해 pose에 대한 error를 구한다.
	x_error.segment<3>(0) = x_cubic - x_;
	x_error.segment<3>(3) = DyrosMath::getPhi(rotation_, rotation_cubic);

	// debug_file_ << xd_desired.transpose() << endl;
	// xd_desired.segment<3>(3).setZero();

	// Feedback gain 설정
	Matrix6d kp;
	kp.setIdentity();
	kp = kp * hz_ * 1.5;	
	//Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * (xd_desired + kp * x_error);

	// joint 값 계산 (CLIK)
	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * xd_desired;
	q_desired_ = q_desired_ + qd_desired / hz_; // q_desired_는 실시간으로 로봇으로 보내진다.
	record(2, duration);
}


// ------------------------------------ HW --------------------------------------- 
void ArmController::hw_2_1(const Vector12d & target_x, double duration)
{
	//------------------------------------------------------------------------
	// Firstly, calculate desired pose and velocity ( end-effector ) for current time
	// by given data ( target pose, duration ).
	// Initial pose is given, initial and final velocity are zero.
	
	Vector6d xd_desired; // v, w
	Vector3d x_desired; // only for position

	// desired linear velocity
	for (int i = 0; i < 3; i++) 
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}

	// Get target rotation matrix
	Matrix3d rotation;
	for (int i = 0; i < 3; i++) 
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i * 3);
	}

	// desired angular velocity
	xd_desired.tail(3) = DyrosMath::rotationCubicDot(play_time_, control_start_time_,
		control_start_time_ + duration, Vector3d::Zero(), Vector3d::Zero(), rotation_init_, rotation);

	// desired position
	for (int i = 0; i < 3; i++)
	{
		x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}

	// desired orientation
	Matrix3d rotation_desired = DyrosMath::rotationCubic(play_time_, control_start_time_,
		control_start_time_ + duration, rotation_init_, rotation);
	//---------------------------------------------------------------------------

	//---------------------------------------------------------------------------
	// After calcuate desired pose and velocity, 
	// calculate desired joint velocity by simple jacobian algorithm.

	// Get jacobian from desired joint value, not current joint value
	Matrix<double, 6, 7> j_qd = jacobianFromqd(0);

	// desired joint velocity and joint position
	Vector7d qd_desired = j_qd.transpose() * (j_qd*j_qd.transpose()).inverse() * xd_desired;
	q_desired_ = q_desired_ + qd_desired / hz_; 
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ current position, desired position, joint position ]
	recordHW2(4, duration, x_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_2_2(const Vector12d & target_x, double duration)
{
	//------------------------------------------------------------------------
	// Firstly, calculate desired pose and velocity ( end-effector ) for current time
	// by given data ( target pose, duration ).
	// Initial pose is given, initial and final velocity are zero.
	
	Vector6d xd_desired; // v, w
	Vector3d x_desired; // only for position

	// desired linear velocity
	for (int i = 0; i < 3; i++) 
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}

	// Get target rotation matrix
	Matrix3d rotation;
	for (int i = 0; i < 3; i++) 
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i * 3);
	}

	// desired angular velocity
	xd_desired.tail(3) = DyrosMath::rotationCubicDot(play_time_, control_start_time_,
		control_start_time_ + duration, Vector3d::Zero(), Vector3d::Zero(), rotation_init_, rotation);

	// desired position
	for (int i = 0; i < 3; i++)
	{
		x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}

	// desired orientation
	Matrix3d rotation_desired = DyrosMath::rotationCubic(play_time_, control_start_time_,
		control_start_time_ + duration, rotation_init_, rotation);
	//---------------------------------------------------------------------------

	//---------------------------------------------------------------------------
	// After calcuate desired pose and velocity, 
	// calculate desired joint velocity by Closed Loop Inverse Kinematic (CLIK).

	// error of pose 
	Vector6d x_error;
	x_error.head(3) = x_desired - CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
	x_error.tail(3) = DyrosMath::getPhi(CalcBodyWorldOrientation(*model_, q_desired_, body_id_[DOF - 1], false).transpose(), rotation_desired);

	// set feedback gain 
	Vector6d kp_diag;
	kp_diag << 50, 50, 50, 10, 10, 10;
	Matrix6d kp = kp_diag.asDiagonal();

	// desired joint velocity and joint position
	// Get jacobian from desired joint value, not current joint value
	Matrix<double, 6, 7> j_qd = jacobianFromqd(0);
	Vector7d qd_desired = j_qd.transpose() * (j_qd*j_qd.transpose()).inverse() * ( xd_desired + kp * x_error );
	q_desired_ = q_desired_ + qd_desired / hz_; 
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ current position, desired position, joint position ]
	recordHW2(5, duration, x_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_2_3(const Vector12d & target_x, double duration)
{
	//------------------------------------------------------------------------
	// Firstly, calculate desired pose and velocity ( end-effector ) for current time
	// by given data ( target pose, duration ).
	// Initial pose is given, initial and final velocity are zero.
	
	Vector6d xd_desired; // v, w
	Vector3d x_desired; // only for position

	// desired linear velocity
	for (int i = 0; i < 3; i++) 
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}

	// Get target rotation matrix
	Matrix3d rotation;
	for (int i = 0; i < 3; i++) 
	{
		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i * 3);
	}

	// desired angular velocity
	xd_desired.tail(3) = DyrosMath::rotationCubicDot(play_time_, control_start_time_,
		control_start_time_ + duration, Vector3d::Zero(), Vector3d::Zero(), rotation_init_, rotation);

	// desired position
	for (int i = 0; i < 3; i++)
	{
		x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
	}

	// desired orientation
	Matrix3d rotation_desired = DyrosMath::rotationCubic(play_time_, control_start_time_,
		control_start_time_ + duration, rotation_init_, rotation);
	//---------------------------------------------------------------------------

	//---------------------------------------------------------------------------
	// After calcuate desired pose and velocity, 
	// calculate desired joint velocity by Closed Loop Inverse Kinematic (CLIK).

	// error of pose 
	Vector6d x_error;
	x_error.head(3) = x_desired - CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false);
	x_error.tail(3) = DyrosMath::getPhi(CalcBodyWorldOrientation(*model_, q_desired_, body_id_[DOF - 1], false).transpose(), rotation_desired);

	// Set feedback gain 
	Vector6d kp_diag;
	kp_diag << 50, 50, 50, 10, 10, 10;
	Matrix6d kp = kp_diag.asDiagonal();

	// Set weighted matrix 
	Vector7d w_inv;
	w_inv << 1.0, 1.0, 1.0, 0.001, 1.0, 1.0, 1.0;
	Matrix7d W_inv = w_inv.asDiagonal();

	// desired joint velocity and joint position
	// Get jacobian from desired joint value, not current joint value
	Matrix<double, 6, 7> j_qd = jacobianFromqd(0);
	Vector7d qd_desired = W_inv * j_qd.transpose() * (j_qd*W_inv*j_qd.transpose()).inverse() * ( xd_desired + kp * x_error );
	q_desired_ = q_desired_ + qd_desired / hz_; 
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ current position, desired position, joint position ]
	recordHW2(6, duration, x_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_3_1(const Vector6d & target_x, double duration)
{
	//------------------------------------------------------------------------
	// Firstly, calculate desired pose and velocity ( end-effector ) for current time
	// by given data ( target pose, duration ).
	// Initial pose is given, initial and final velocity are zero.
	// The velocity and position of x are of end-effector and COM of link 4.

	Vector6d xd_desired, x_desired; // end-effector, COM of link 4

	for (int i = 0; i < 3; i++) // Desired linear velocity
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,   // End-effector
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);  
		xd_desired(i+3) = DyrosMath::cubicDot(play_time_, control_start_time_, // COM of Link 4 
			control_start_time_ + duration, x_2_init_(i), target_x(i+3), 0, 0);
	}

	for (int i = 0; i < 3; i++) // Desired position
	{
		x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,      // End-effector
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
		x_desired(i+3) = DyrosMath::cubic(play_time_, control_start_time_,    // COM of Link 4 
			control_start_time_ + duration, x_2_init_(i), target_x(i+3), 0, 0);	
			
	}
	//---------------------------------------------------------------------------

	//---------------------------------------------------------------------------
	// After calcuate desired pose and velocity, 
	// calculate desired joint velocity by Closed Loop Inverse Kinematic (CLIK).

	// error position
	Vector6d x_qd, x_error;
	// position(EEF, COM) from desired joint position
	x_qd << CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false),  // End-effector
			CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 4], com_position_[DOF - 4], false);  // COM of Link
	x_error = x_desired - x_qd;

	// Feedback gain
	Vector6d kp_diag;
	kp_diag =100*Vector6d::Ones();
	Matrix6d kp = kp_diag.asDiagonal();

	// desired joint velocity and joint position
	// Get jacobian from desired joint value, not current joint value
	Matrix<double, 6, 7> j_qd;
	j_qd << jacobianFromqd(0).block <3, DOF>(0, 0), jacobianFromqd(1).block < 3, DOF>(0, 0);
	Vector7d qd_desired = j_qd.transpose() * ( j_qd * j_qd.transpose() + 0.01*EYE(6) ).inverse() * ( xd_desired + kp * x_error );
	q_desired_ = q_desired_ + qd_desired / hz_;
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ current position(EEF, COM), desired position(EEF, COM), joint position ]
	recordHw3(7, duration, x_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_3_2(const Vector6d & target_x, double duration)
{
	//------------------------------------------------------------------------
	// Firstly, calculate desired pose and velocity ( end-effector ) for current time
	// by given data ( target pose, duration ).
	// Initial pose is given, initial and final velocity are zero.
	// The velocity and position of x are of end-effector and COM of link 4.

	Vector6d xd_desired, x_desired; // end-effector, COM of link 4

	for (int i = 0; i < 3; i++) // Desired linear velocity
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,   // End-effector
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);  
		xd_desired(i+3) = DyrosMath::cubicDot(play_time_, control_start_time_, // COM of Link 4 
			control_start_time_ + duration, x_2_init_(i), target_x(i+3), 0, 0);
	}

	for (int i = 0; i < 3; i++) // Desired position
	{
		x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,      // End-effector
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
		x_desired(i+3) = DyrosMath::cubic(play_time_, control_start_time_,    // COM of Link 4 
			control_start_time_ + duration, x_2_init_(i), target_x(i+3), 0, 0);	
			
	}
	//---------------------------------------------------------------------------

	//---------------------------------------------------------------------------
	// After calcuate desired pose and velocity, 
	// calculate desired joint velocity by Closed Loop Inverse Kinematic (CLIK) with Null-space.

	Vector6d x_qd, xd_CLIK;
	// position(EEF, COM) from desired joint position
	x_qd << CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false), // End-effector
			CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 4], com_position_[DOF - 4], false); // COM of Link 4

	// Feedback gain 
	Vector3d kp_diag;
	kp_diag =10*Vector3d::Ones();
	Matrix3d kp = kp_diag.asDiagonal();	

	// x_dot from CLIK
	xd_CLIK.head(3) = xd_desired.head(3) + kp * ( x_desired.head(3) - x_qd.head(3) ); 
	xd_CLIK.tail(3) = xd_desired.tail(3) + kp * ( x_desired.tail(3) - x_qd.tail(3) );

	// Get Jacobian and pseudo-inverse Jacobian 
	// Get jacobian from desired joint value, not current joint value
	Matrix<double, 3, 7> j_qd_1, j_qd_2;
	Matrix<double, 7, 3>j_pseudo_1, j_pseudo_2;
	j_qd_1 = jacobianFromqd(0).block <3, DOF>(0, 0); // End-effector
	j_qd_2 = jacobianFromqd(1).block <3, DOF>(0, 0); // COM of Link 4
	j_pseudo_1 = j_qd_1.transpose() * ( j_qd_1 * j_qd_1.transpose() ).inverse(); // End-effector
	j_pseudo_2 = j_qd_2.transpose() * ( j_qd_2 * j_qd_2.transpose() ).inverse(); // COM of Link 4

	// Null-space
	Matrix7d Null = EYE(7) - j_pseudo_1 * j_qd_1;

	// desired joint velocity and joint position
	Vector7d qd_2desired = j_pseudo_2 * ( xd_CLIK.tail(3) - j_qd_2 * j_pseudo_1 * xd_CLIK.head(3) );
	Vector7d qd_desired  = j_pseudo_1 * xd_CLIK.head(3) + Null * qd_2desired; 
	q_desired_ = q_desired_ + qd_desired / hz_; 
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ current position(EEF, COM), desired position(EEF, COM), joint position ]
	recordHw3(8, duration, x_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_3_3(const Vector6d & target_x, double duration, bool isStep)
{
	//------------------------------------------------------------------------
	//------------------------------------------------------------------------
	// Firstly, calculate desired pose and velocity ( end-effector ) for current time
	// by given data ( target pose, duration ).
	// Initial pose is given, initial and final velocity are zero.
	// The velocity and position of x are of end-effector and COM of link 4.

	Vector6d xd_desired, x_desired; // end-effector, COM of link 4

	for (int i = 0; i < 3; i++) // Desired linear velocity
	{
		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,   // End-effector
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);  
		xd_desired(i+3) = DyrosMath::cubicDot(play_time_, control_start_time_, // COM of Link 4 
			control_start_time_ + duration, x_2_init_(i), target_x(i+3), 0, 0);
	}

	for (int i = 0; i < 3; i++) // Desired position
	{
		x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,      // End-effector
			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
		x_desired(i+3) = DyrosMath::cubic(play_time_, control_start_time_,    // COM of Link 4 
			control_start_time_ + duration, x_2_init_(i), target_x(i+3), 0, 0);	
			
	}
	//---------------------------------------------------------------------------

	//---------------------------------------------------------------------------
	// After calcuate desired pose and velocity, 
	// calculate desired joint velocity by Closed Loop Inverse Kinematic (CLIK) 
	// with Null-space and transition algorithm.
	
	Vector6d x_qd, xd_CLIK;
	// position(EEF, COM) from desired joint position
	x_qd << CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 1], com_position_[DOF - 1], false), // End-effector
			CalcBodyToBaseCoordinates(*model_, q_desired_, body_id_[DOF - 4], com_position_[DOF - 4], false); // COM of Link 4

	// Feedback gain 
	Vector3d kp_diag;
	kp_diag =10*Vector3d::Ones();
	Matrix3d kp = kp_diag.asDiagonal();	

	// x_dot from CLIK
	xd_CLIK.head(3) = xd_desired.head(3) + kp * ( x_desired.head(3) - x_qd.head(3) ); 
	xd_CLIK.tail(3) = xd_desired.tail(3) + kp * ( x_desired.tail(3) - x_qd.tail(3) );

	// Get Jacobian and pseudo-inverse Jacobian 
	// Get jacobian from desired joint value, not current joint value
	Matrix<double, 3, 7> j_qd_1, j_qd_2;
	Matrix<double, 7, 3>j_pseudo_1, j_pseudo_2;
	j_qd_1 = jacobianFromqd(0).block <3, DOF>(0, 0); // End-effector
	j_qd_2 = jacobianFromqd(1).block <3, DOF>(0, 0); // COM of Link 4
	j_pseudo_1 = j_qd_1.transpose() * ( j_qd_1 * j_qd_1.transpose() ).inverse(); // End-effector
	j_pseudo_2 = j_qd_2.transpose() * ( j_qd_2 * j_qd_2.transpose() ).inverse(); // COM of Link 4

	// Null-space
	Matrix7d Null = EYE(7) - j_pseudo_1 * j_qd_1;

	// Parameter for task transition(h)
	double h_1, h_2;
	h_1 = 1.0;
	if(isStep)
	{
		if(play_time_ > (control_start_time_ + duration/2)) h_2 = 1.0;
		else                                                h_2 = 0.0;
	}
	else
	{
		if(play_time_ > control_start_time_ + duration)     h_2 = 1.0;
		else if(play_time_ < control_start_time_)           h_2 = 0.0;
		else                                                h_2 = (play_time_ - control_start_time_) / duration;
	}


	Vector3d xd_i_1, xd_i_2;
	xd_i_1 = h_1 * xd_CLIK.head(3) + (1-h_1) * j_qd_1 *j_pseudo_2 * h_2 * xd_CLIK.tail(3);
	xd_i_2 = h_2 * xd_CLIK.tail(3) + (1-h_2) * j_qd_2 *j_pseudo_1 * h_1 * xd_CLIK.head(3); 

	// desired joint velocity and joint position
	Vector7d qd_desired  = j_pseudo_1 * xd_i_1 + Null * j_pseudo_2 * ( xd_i_2 - j_qd_2 * j_pseudo_1 * xd_i_1 ); 
	q_desired_ = q_desired_ + qd_desired / hz_; 
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ current position(EEF, COM), desired position(EEF, COM), joint position ]
	if(isStep) recordHw3(9, duration, x_desired);
	else recordHw3(10, duration, x_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_4_1(const Vector7d &target_q, double duration)
{
	// Simple PD controller

	// Gain Matrix
	Vector7d kp_diag, kv_diag;
	Matrix7d kp, kv;
	kp_diag = 30 * Vector7d::Ones();
	kv_diag = 1 * Vector7d::Ones();
	kp = kp_diag.asDiagonal();
	kv = kv_diag.asDiagonal();

	// desired joint value
	Vector7d q_desired, qd_desired;
	qd_desired = Vector7d::Zero();
	if(play_time_ < control_start_time_ + duration/2)  q_desired = q_init_;
	else q_desired = target_q;

	// Apply force
	torque_desired_ = kp*( q_desired - q_ ) + kv*( qd_desired - qdot_ );

	// record
	recordHw4(11, duration, q_desired);
}

void ArmController::hw_4_2(const Vector7d &target_q, double duration, bool isStep)
{
	// Simple PD controller

	// Gain Matrix
	Vector7d kp_diag, kv_diag;
	Matrix7d kp, kv;
	kp_diag = 30 * Vector7d::Ones();
	kv_diag = 1 * Vector7d::Ones();
	kp = kp_diag.asDiagonal();
	kv = kv_diag.asDiagonal();

	// desired joint value
	Vector7d q_desired, qd_desired;
	if(isStep)
	{
		qd_desired = Vector7d::Zero();
		if(play_time_ < control_start_time_ + duration/2)  q_desired = q_init_;
		else q_desired = target_q;
	}
	else
	{
		for (int i = 0; i < 7; i++)
		{
			qd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
				control_start_time_ + duration, q_init_(i), target_q(i), 0, 0);
			q_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,
				control_start_time_ + duration, q_init_(i), target_q(i), 0, 0);
		}
	}

	// Apply force
	torque_desired_ = ( kp*( q_desired - q_ ) + kv*( qd_desired - qdot_ ) ) + g_;

	// record
	recordHw4(12, duration, q_desired);
}

void ArmController::hw_4_3(const Vector7d &target_q, double duration, bool isStep)
{
	// Simple PD controller

	// Gain Matrix
	Vector7d kp_diag, kv_diag;
	Matrix7d kp, kv;
	kp_diag = 400 * Vector7d::Ones();
	kv_diag = 40 * Vector7d::Ones();
	kp = kp_diag.asDiagonal();
	kv = kv_diag.asDiagonal();

	// desired joint value
	Vector7d q_desired, qd_desired;
	if(isStep)
	{
		qd_desired = Vector7d::Zero();
		if(play_time_ < control_start_time_ + duration/2)  q_desired = q_init_;
		else q_desired = target_q;
	}
	else
	{
		for (int i = 0; i < 7; i++)
		{
			qd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
				control_start_time_ + duration, q_init_(i), target_q(i), 0, 0);
			q_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,
				control_start_time_ + duration, q_init_(i), target_q(i), 0, 0);
		}
	}


	

	// Apply force
	torque_desired_ = m_ * ( kp*( q_desired - q_ ) + kv*( qd_desired - qdot_ ) ) + g_;

	// record
	if(isStep) recordHw4(13, duration, q_desired);
	else recordHw4(14, duration, q_desired);
	
}

void ArmController::hw_5_1(const Vector12d &target_x, double duration, bool isStep)
{
	double kp, kv;
	kp = 400.0;
	kv = 40.0;

	Vector3d x_desired; // pose, orientation
	Matrix3d rotation_desired;
	Vector6d xd_desired, x_error; // v, w
	
	if(isStep)
	{
		xd_desired = Vector6d::Zero();
		if(play_time_ < control_start_time_ + duration/2)
		{
			x_desired = x_init_;
			rotation_desired = rotation_init_;
		}
		else
		{
			x_desired = target_x.head(3);
			rotation_desired << target_x.tail(9);
		}
	}
	else
	{
		for (int i = 0; i < 3; i++) // 현재 시간에 맞는 desired linear velocity를 구한다.
		{
			xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
				control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
		}
		Matrix3d rotation;

		for (int i = 0; i < 3; i++) // 현재 시간에 맞는 desired angular velocity를 구한다.
		{
			rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i * 3);
		}
		xd_desired.segment<3>(3) = DyrosMath::rotationCubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, Vector3d::Zero(), Vector3d::Zero(), rotation_init_, rotation);

		for (int i = 0; i < 3; i++) // 현재 시간에 맞는 desired position을 구한다.
		{
			x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,
				control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
		}

		// 현재 시간에 맞는 desired ratation을 구한다.
		rotation_desired = DyrosMath::rotationCubic(play_time_, control_start_time_,
			control_start_time_ + duration, rotation_init_, rotation);
	}
	
	Vector6d control_force; // force, momentum
	x_error.segment<3>(0) = x_desired - x_;	
	x_error.segment<3>(3) = DyrosMath::getPhi(rotation_, rotation_desired);
	control_force.segment<3>(0) = kp * x_error.head(3) + kv * (xd_desired.head(3) - x_dot_.head(3));
	control_force.segment<3>(3) = -kp * x_error.tail(3) - kv * xd_desired.tail(3);

	Vector7d null_torque = m_ * ( kp * (q_init_ - q_) - kv * qdot_ );
	Matrix6d PseudoKine = ( j_ * m_.inverse() * j_.transpose() ).inverse();
	Matrix<double, 7, 6> GeneralInvJ = m_.inverse() * j_.transpose() * PseudoKine;


	torque_desired_ = j_.transpose() * PseudoKine * control_force + ( EYE(7) - j_.transpose() * GeneralInvJ.transpose() ) * null_torque +  g_;

	if(isStep) recordHw5(15, duration, x_desired, xd_desired);
	else recordHw5(16, duration, x_desired, xd_desired);
}

void ArmController::hw_5_2(const Vector12d &target_x, double duration)
{
	double kp, kv;
	kp = 400.0;
	kv = 40.0;

	Vector3d x_desired; // pose, orientation
	Matrix3d rotation_desired;
	Vector6d xd_desired, x_error; // v, w
	double xd_max = 0.3;
	
	if(play_time_ < control_start_time_ + duration/2)
	{
		x_desired = x_init_;
		rotation_desired = rotation_init_;
	}
	else
	{
		x_desired = target_x.head(3);
		rotation_desired << target_x.tail(9);
	}

	if( ( kp/kv * (x_desired - x_) ).norm() < xd_max )
	{
		xd_desired.head(3) = kp/kv * (x_desired - x_);
	}
	else
	{
		xd_desired.head(3) = xd_max / (x_desired - x_).norm() * (x_desired - x_);
	}
	xd_desired.tail(3) = Vector3d::Zero();
	
	Vector6d control_force; // force, momentum
	// x_error.segment<3>(0) = x_desired - x_;	
	x_error.segment<3>(3) = DyrosMath::getPhi(rotation_, rotation_desired);
	control_force.segment<3>(0) = kv * (xd_desired.head(3) - x_dot_.head(3));
	control_force.segment<3>(3) = -kp * x_error.tail(3) - kv * xd_desired.tail(3);

	Vector7d null_torque = m_ * ( kp * (q_init_ - q_) - kv * qdot_ );
	Matrix6d PseudoKine = ( j_ * m_.inverse() * j_.transpose() ).inverse();
	Matrix<double, 7, 6> GeneralInvJ = m_.inverse() * j_.transpose() * PseudoKine;


	torque_desired_ = j_.transpose() * PseudoKine * control_force + ( EYE(7) - j_.transpose() * GeneralInvJ.transpose() ) * null_torque +  g_;

	recordHw5(17, duration, x_desired, xd_desired);
}

void ArmController::hw_7(const Vector12d &target_x, double duration)
{
	double kp, kv;
	kp = 400.0;
	kv = 40.0;

	Vector3d x_desired; // pose, orientation
	Matrix3d rotation_desired;
	Vector6d xd_desired, x_error; // v, w
	double xd_max = 0.3;
	
	if(play_time_ < control_start_time_ + duration/2)
	{
		x_desired = x_init_;
		rotation_desired = rotation_init_;
	}
	else
	{
		x_desired = target_x.head(3);
		rotation_desired << target_x.tail(9);
	}

	if( ( kp/kv * (x_desired - x_) ).norm() < xd_max )
	{
		xd_desired.head(3) = kp/kv * (x_desired - x_);
	}
	else
	{
		xd_desired.head(3) = xd_max / (x_desired - x_).norm() * (x_desired - x_);
	}
	xd_desired.tail(3) = Vector3d::Zero();

	Vector3d x_obs;
	x_obs << 0.15, -0.012, 0.65; 
	double dist_obs, dist_0, k_obs;
	dist_obs = (x_ - x_obs).norm();
	dist_0 = 0.15;
	k_obs = 0.1;
	Vector3d rep_force = k_obs * (1/dist_obs - 1/dist_0) * pow(dist_obs, -3) * (x_ - x_obs);
	
	Vector6d control_force; // force, momentum
	// x_error.segment<3>(0) = x_desired - x_;	
	x_error.segment<3>(3) = DyrosMath::getPhi(rotation_, rotation_desired);
	control_force.segment<3>(0) = kv * (xd_desired.head(3) - x_dot_.head(3)) + rep_force;
	control_force.segment<3>(3) = -kp * x_error.tail(3) - kv * xd_desired.tail(3);

	Vector7d null_torque = m_ * ( kp * (q_init_ - q_) - kv * qdot_ );
	Matrix6d PseudoKine = ( j_ * m_.inverse() * j_.transpose() ).inverse();
	Matrix<double, 7, 6> GeneralInvJ = m_.inverse() * j_.transpose() * PseudoKine;


	torque_desired_ = j_.transpose() * PseudoKine * control_force + ( EYE(7) - j_.transpose() * GeneralInvJ.transpose() ) * null_torque +  g_;

	recordHw5(18, duration, x_desired, xd_desired);
}

// Controller Core Methods ----------------------------

void ArmController::setMode(const std::string & mode)
{
	is_mode_changed_ = true;
	control_mode_ = mode;
	cout << "Current mode (changed) : " << mode << endl;
}
void ArmController::initDimension()
{
	dof_ = DOF;
	q_temp_.resize(DOF);
	j_temp_.resize(6, DOF);

	qddot_.setZero();

	x_target_.setZero();
	q_desired_.setZero();
	torque_desired_.setZero();

	g_temp_.resize(DOF);
	m_temp_.resize(DOF, DOF);

	j_temp_2_.resize(6, DOF);
	j_temp_2_.setZero();
}

void ArmController::initModel()
{
    model_ = make_shared<Model>();

    model_->gravity = Vector3d(0., 0, -GRAVITY);

    double mass[DOF];
    mass[0] = 1.0;
    mass[1] = 1.0;
    mass[2] = 1.0;
    mass[3] = 1.0;
    mass[4] = 1.0;
    mass[5] = 1.0;
    mass[6] = 1.0;

    Vector3d axis[DOF];
	axis[0] = Eigen::Vector3d::UnitZ();
	axis[1] = Eigen::Vector3d::UnitY();
	axis[2] = Eigen::Vector3d::UnitZ();
	axis[3] = -1.0*Eigen::Vector3d::UnitY();
	axis[4] = Eigen::Vector3d::UnitZ();
	axis[5] = -1.0*Eigen::Vector3d::UnitY();
	axis[6] = -1.0*Eigen::Vector3d::UnitZ();


	Eigen::Vector3d global_joint_position[DOF];

	global_joint_position[0] = Eigen::Vector3d(0.0, 0.0, 0.3330);
	global_joint_position[1] = global_joint_position[0];
	global_joint_position[2] = Eigen::Vector3d(0.0, 0.0, 0.6490);
	global_joint_position[3] = Eigen::Vector3d(0.0825, 0.0, 0.6490);
	global_joint_position[4] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[5] = Eigen::Vector3d(0.0, 0.0, 1.0330);
	global_joint_position[6] = Eigen::Vector3d(0.0880, 0.0, 1.0330);

	joint_posision_[0] = global_joint_position[0];
	for (int i = 1; i < DOF; i++)
		joint_posision_[i] = global_joint_position[i] - global_joint_position[i - 1];

	com_position_[0] = Vector3d(0.000096, -0.0346, 0.2575);
	com_position_[1] = Vector3d(0.0002, 0.0344, 0.4094);
	com_position_[2] = Vector3d(0.0334, 0.0266, 0.6076);
	com_position_[3] = Vector3d(0.0331, -0.0266, 0.6914);
	com_position_[4] = Vector3d(0.0013, 0.0423, 0.9243);
	com_position_[5] = Vector3d(0.0421, -0.0103, 1.0482);
	com_position_[6] = Vector3d(0.1, -0.0120, 0.9536);

	for (int i = 0; i < DOF; i++)
		com_position_[i] -= global_joint_position[i];

    Math::Vector3d inertia[DOF];
	for (int i = 0; i < DOF; i++)
		inertia[i] = Eigen::Vector3d::Identity() * 0.001;

    for (int i = 0; i < DOF; i++) {
        body_[i] = Body(mass[i], com_position_[i], inertia[i]);
        joint_[i] = Joint(JointTypeRevolute, axis[i]);
        if (i == 0)
            body_id_[i] = model_->AddBody(0, Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
        else
            body_id_[i] = model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_posision_[i]), joint_[i], body_[i]);
    }
}

void ArmController::initFile()
{
	debug_file_.open("debug.txt");
	for (int i = 0; i < NUM_HW_PLOT; i++)
	{
		hw_plot_files_[i].open(hw_plot_file_names_[i] + ".txt");
	}
}

void ArmController::readData(const Vector7d &position, const Vector7d &velocity, const Vector7d &torque)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = torque(i);
	}
}
void ArmController::readData(const Vector7d &position, const Vector7d &velocity)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = 0;
	}
}

const Vector7d & ArmController::getDesiredPosition()
{
	return q_desired_;
}

const Vector7d & ArmController::getDesiredTorque()
{
	return torque_desired_;
}



void ArmController::initPosition()
{
    q_init_ = q_;
    q_desired_ = q_init_;
}

// ----------------------------------------------------

