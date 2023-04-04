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
		// moveJointPositionTorque(target_position, 1.0);     
		moveJointPosition(target_position, 1.0);                
	}
	else if (control_mode_ == "hw_3_1")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
					0, -1, 0,
					-1, 0, 0,
					0, 0, -1;
		hw_3_1(target_x, 4.0);
	}
	else if (control_mode_ == "hw_3_2")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
					0, -1, 0,
					-1, 0, 0,
					0, 0, -1;
		hw_3_2(target_x, 4.0);
	}
	else if (control_mode_ == "hw_3_3")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
					0, -1, 0,
					-1, 0, 0,
					0, 0, -1;
		hw_3_3(target_x, 4.0);
	}
	else if (control_mode_ == "hw_3_4")
	{
		Vector12d target_x;
		target_x << 0.25, 0.28, 0.65,
					0, -1, 0,
					-1, 0, 0,
					0, 0, -1;
		hw_3_4(target_x, 4.0);
	}
	else if (control_mode_ == "hw_4_1")
	{
		Vector6d target_x;
		target_x << 0.25,  0.28, 0.65, // for end-effector
					0.00, -0.15, 0.60; // for link4
		hw_4_1(target_x, 4.0);
	}
	else if (control_mode_ == "hw_4_2")
	{
		Vector6d target_x;
		target_x << 0.25,  0.28, 0.65, // for end-effector
					0.00, -0.15, 0.60; // for link4
		hw_4_2(target_x, 4.0);
	}
	else if (control_mode_ == "hw_5_1")
	{
		Vector7d target_q;
		target_q << 0.0, 0.0, 0.0, -M_PI*(25./180.), 0.0, M_PI / 2, 0; 
		hw_5_1(target_q, 4.0);
	}
	else if (control_mode_ == "hw_5_2_1")
	{
		Vector7d target_q;
		target_q << 0.0, 0.0, 0.0, -M_PI*(25./180.), 0.0, M_PI / 2, 0; 
		hw_5_2_1(target_q, 4.0);
	}
	else if (control_mode_ == "hw_5_2_2")
	{
		Vector7d target_q;
		target_q << 0.0, 0.0, 0.0, -M_PI/3., 0.0, M_PI / 2, 0; 
		hw_5_2_2(target_q, 4.0);
	}
	else if (control_mode_ == "hw_5_3_1")
	{
		Vector7d target_q;
		target_q << 0.0, 0.0, 0.0, -M_PI*(25./180.), 0.0, M_PI / 2, 0;
		hw_5_3_1(target_q, 4.0);
	}
	else if (control_mode_ == "hw_5_3_2")
	{
		Vector7d target_q;
		target_q << 0.0, 0.0, 0.0, -M_PI/3., 0.0, M_PI / 2, 0; 
		hw_5_3_2(target_q, 4.0);
	}
	// else if (control_mode_ == "hw_6_1_1")
	// {
	// 	Vector12d target_x;
	// 	target_x << x_init_, rotation_init_.col(0), rotation_init_.col(1), rotation_init_.col(2);
	// 	target_x(1) += 0.02;
	// 	hw_6_1_1(target_x, 4.0);
	// }
	// else if (control_mode_ == "hw_6_1_2")
	// {
	// 	Vector12d target_x;
	// 	target_x << x_init_, rotation_init_.col(0), rotation_init_.col(1), rotation_init_.col(2);
	// 	target_x(1) += 0.1; 
	// 	hw_6_1_2(target_x, 4.0);
	// }
	// else if (control_mode_ == "hw_6_2_1")
	// {
	// 	Vector12d target_x;
	// 	target_x << x_init_, rotation_init_.col(0), rotation_init_.col(1), rotation_init_.col(2);
	// 	target_x(1) += 0.02;
	// 	hw_6_2_1(target_x, 4.0);
	// }
	// else if (control_mode_ == "hw_6_2_2")
	// {
	// 	Vector12d target_x;
	// 	target_x << x_init_, rotation_init_.col(0), rotation_init_.col(1), rotation_init_.col(2);
	// 	target_x(1) += 0.1; 
	// 	hw_6_2_2(target_x, 4.0);
	// }
	else
	{
		torque_desired_ = g_;
	}

	printState();

	tick_++;
	play_time_ = tick_ / hz_;	// second
}

// void ArmController::record(int file_number, double duration)
// {
// 	if (play_time_ < control_start_time_ + duration + 1.0)
// 	{
// 		hw_plot_files_[file_number] << x_.transpose() <<
// 			Map< Matrix<double, 1, 9> >(rotation_.data(), rotation_.size()) << x_cubic_.transpose() <<
// 			endl;
// 	}
// }

// void ArmController::record(int file_number, double duration, const stringstream & ss)
// {
// 	if (play_time_ < control_start_time_ + duration + 1.0)
// 	{
// 		hw_plot_files_[file_number] << ss.str() << endl;
// 	}
// }

void ArmController::recordHW3(int file_number, double duration, const Vector3d & x_desired, const Matrix3d & rotation_desired)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] 
		<< x_desired.transpose()<< " " << x_.transpose() << " " 
		<< rotation_desired.col(0).transpose() << " " << rotation_desired.col(1).transpose() << " " << rotation_desired.col(2).transpose() << " "
		<< rotation_.col(0).transpose() << " " << rotation_.col(1).transpose() << " " << rotation_.col(2).transpose() << " " 
		<< q_.transpose()
		<< endl;
	}
}

void ArmController::recordHw4(int file_number, double duration, const Vector6d &x_desired)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] 
		<< x_desired.transpose() <<  " "
		<< x_.transpose()<< " " << x_2_.transpose()<< " "
		<< q_.transpose()
		<< endl;
	}
}

void ArmController::recordHw5(int file_number, double duration, const Vector7d & q_desired)
{
	if (play_time_ < control_start_time_ + duration + 1.0)
	{
		hw_plot_files_[file_number] 
		<< q_desired.transpose() << " "
		<< q_.transpose() <<  " "
		<< endl;
	}
}

// void ArmController::recordHW6(int file_number, double duration, const Vector3d & x_desired, const Matrix3d & rotation_desired)
// {
// 	if (play_time_ < control_start_time_ + duration + 1.0)
// 	{
// 		hw_plot_files_[file_number] 
// 		<< x_desired.transpose()<< " " << x_.transpose() << " " 
// 		<< rotation_desired.col(0).transpose() << " " << rotation_desired.col(1).transpose() << " " << rotation_desired.col(2).transpose() << " "
// 		<< rotation_.col(0).transpose() << " " << rotation_.col(1).transpose() << " " << rotation_.col(2).transpose() << " " 
// 		<< q_.transpose()
// 		<< endl;
// 	}
// }

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

// ------------------------------------ HW --------------------------------------- 
void ArmController::hw_3_1(const Vector12d & target_x, double duration)
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

	// desired joint velocity and joint position
	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * xd_desired;
	q_desired_ = q_ + qd_desired / hz_; 
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ current position, desired position, desired orientation, current orientation, joint position ]
	recordHW3(0, duration, x_desired, rotation_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_3_2(const Vector12d & target_x, double duration)
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
	x_error.head(3) = x_desired - x_;
	x_error.tail(3) = DyrosMath::getPhi(rotation_desired, rotation_);


	// desired joint velocity and joint position
	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * x_error;
	q_desired_ = q_ + qd_desired / hz_; 
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ current position, desired position, desired orientation, current orientation, joint position ]
	recordHW3(1, duration, x_desired, rotation_desired);
	// ----------------------------------------------------------------------------
}


void ArmController::hw_3_3(const Vector12d & target_x, double duration)
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
	x_error.head(3) = x_desired - x_;
	x_error.tail(3) = DyrosMath::getPhi(rotation_desired, rotation_);

	// Set feedback gain 
	Vector6d kp_diag;
	kp_diag << 50, 50, 50, 10, 10, 10;
	Matrix6d kp = kp_diag.asDiagonal();


	// desired joint velocity and joint position
	Vector7d qd_desired = j_.transpose() * (j_*j_.transpose()).inverse() * (xd_desired + kp*x_error);
	q_desired_ = q_ + qd_desired / hz_; 
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ current position, desired position, desired orientation, current orientation, joint position ]
	recordHW3(2, duration, x_desired, rotation_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_3_4(const Vector12d & target_x, double duration)
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
	x_error.head(3) = x_desired - x_;
	x_error.tail(3) = DyrosMath::getPhi(rotation_desired, rotation_);

	// Set feedback gain 
	Vector6d kp_diag;
	kp_diag << 50, 50, 50, 10, 10, 10;
	Matrix6d kp = kp_diag.asDiagonal();

	// Set weighted matrix 
	Vector7d w_inv;
	w_inv << 1.0, 1.0, 1.0, 0.01, 1.0, 1.0, 1.0;
	Matrix7d W_inv = w_inv.asDiagonal();

	// desired joint velocity and joint position
	Vector7d qd_desired = W_inv * j_.transpose() * (j_*W_inv*j_.transpose()).inverse() * ( xd_desired + kp * x_error );
	q_desired_ = q_desired_ + qd_desired / hz_; 
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ current position, desired position, desired orientation, current orientation, joint position ]
	recordHW3(3, duration, x_desired, rotation_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_4_1(const Vector6d & target_x, double duration)
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
	Vector6d x, x_error;
	// position(EEF, COM) from desired joint position
	x << x_,// End-effector
		 x_2_;  // COM of Link
	x_error = x_desired - x;

	// Feedback gain
	Vector6d kp_diag;
	kp_diag =100*Vector6d::Ones();
	Matrix6d kp = kp_diag.asDiagonal();

	// desired joint velocity and joint position
	Matrix<double, 6, 7> j;
	j << j_v_, j_2_.block < 3, DOF>(0, 0);
	Vector7d qd_desired = j.transpose() * ( j * j.transpose() + 0.01*EYE(6) ).inverse() * ( xd_desired + kp * x_error );
	q_desired_ = q_ + qd_desired / hz_;
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ desired position(EEF, COM),current position(EEF, COM),  joint position ]
	recordHw4(4, duration, x_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_4_2(const Vector6d & target_x, double duration)
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

	Vector6d x, xd_CLIK;
	// position(EEF, COM) from desired joint position
	x << x_, // End-effector
		 x_2_; // COM of Link 4

	// Feedback gain 
	Vector6d kp_diag;
	kp_diag =10*Vector6d::Ones();
	Matrix6d kp = kp_diag.asDiagonal();	

	// x_dot from CLIK
	xd_CLIK = xd_desired + kp * ( x_desired - x ); 

	// Get Jacobian and pseudo-inverse Jacobian 
	Matrix<double, 3, 7> j_1, j_2;
	Matrix<double, 7, 3>j_pseudo_1, j_pseudo_2;
	j_1 = j_v_; // End-effector
	j_2 = j_2_.block <3, DOF>(0, 0); // COM of Link 4
	j_pseudo_1 = j_1.transpose() * ( j_1 * j_1.transpose() ).inverse(); // End-effector
	j_pseudo_2 = j_2.transpose() * ( j_2 * j_2.transpose() ).inverse(); // COM of Link 4

	// Null-space
	Matrix7d Null = EYE(7) - j_pseudo_1 * j_1;

	// desired joint velocity and joint position
	Vector7d qd_2desired = j_pseudo_2 * ( xd_CLIK.tail(3) - j_2 * j_pseudo_1 * xd_CLIK.head(3) );
	Vector7d qd_desired  = j_pseudo_1 * xd_CLIK.head(3) + Null * qd_2desired; 
	q_desired_ = q_ + qd_desired / hz_; 
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ desired position(EEF, COM), current position(EEF, COM), joint position ]
	recordHw4(5, duration, x_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_5_1(const Vector7d &target_q, double duration)
{
	// Simple PD controller
	//---------------------------------------------------------------------------
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
	//---------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ desired joint position, current joint position]
	recordHw5(6, duration, q_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_5_2_1(const Vector7d &target_q, double duration)
{
	// Simple PD controller
	//---------------------------------------------------------------------------
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
	torque_desired_ = ( kp*( q_desired - q_ ) + kv*( qd_desired - qdot_ ) ) + g_;
	//---------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ desired joint position, current joint position]
	recordHw5(7, duration, q_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_5_2_2(const Vector7d &target_q, double duration)
{
	// Simple PD controller
	// ----------------------------------------------------------------------------
	// Gain Matrix
	Vector7d kp_diag, kv_diag;
	Matrix7d kp, kv;
	kp_diag = 30 * Vector7d::Ones();
	kv_diag = 1 * Vector7d::Ones();
	kp = kp_diag.asDiagonal();
	kv = kv_diag.asDiagonal();

	// desired joint value
	Vector7d q_desired, qd_desired;
	for (int i = 0; i < 7; i++)
	{
		qd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_q(i), 0, 0);
		q_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_q(i), 0, 0);
	}

	// Apply force
	torque_desired_ = ( kp*( q_desired - q_ ) + kv*( qd_desired - qdot_ ) ) + g_;
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ desired joint position, current joint position]
	recordHw5(8, duration, q_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_5_3_1(const Vector7d &target_q, double duration)
{
	// Simple PD controller
	// ----------------------------------------------------------------------------
	// Gain Matrix
	Vector7d kp_diag, kv_diag;
	Matrix7d kp, kv;
	kp_diag = 400 * Vector7d::Ones();
	kv_diag = 40 * Vector7d::Ones();
	kp = kp_diag.asDiagonal();
	kv = kv_diag.asDiagonal();

	// desired joint value
	Vector7d q_desired, qd_desired;
	qd_desired = Vector7d::Zero();
	if(play_time_ < control_start_time_ + duration/2)  q_desired = q_init_;
	else q_desired = target_q;

	// Apply force
	torque_desired_ = m_ * ( kp*( q_desired - q_ ) + kv*( qd_desired - qdot_ ) ) + g_;
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ desired joint position, current joint position]
	recordHw5(9, duration, q_desired);
	// ----------------------------------------------------------------------------
}

void ArmController::hw_5_3_2(const Vector7d &target_q, double duration)
{
	// Simple PD controller
	// ----------------------------------------------------------------------------
	// Gain Matrix
	Vector7d kp_diag, kv_diag;
	Matrix7d kp, kv;
	kp_diag = 400 * Vector7d::Ones();
	kv_diag = 40 * Vector7d::Ones();
	kp = kp_diag.asDiagonal();
	kv = kv_diag.asDiagonal();

	// desired joint value
	Vector7d q_desired, qd_desired;
	for (int i = 0; i < 7; i++)
	{
		qd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_q(i), 0, 0);
		q_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, q_init_(i), target_q(i), 0, 0);
	}

	// Apply force
	torque_desired_ = m_ * ( kp*( q_desired - q_ ) + kv*( qd_desired - qdot_ ) ) + g_;
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ desired joint position, current joint position]
	recordHw5(10, duration, q_desired);
	// ----------------------------------------------------------------------------
}

// void ArmController::hw_6_1_1(const Vector12d &target_x, double duration)
// {
// 	Vector3d x_desired; // position
// 	Matrix3d rotation_desired; // orientation
// 	Vector6d xd_desired, x_error, xd_error; // v, w
	
// 	// Desired trajectory
// 	// ----------------------------------------------------------------------------
// 	xd_desired = Vector6d::Zero();
// 	if(play_time_ < control_start_time_ + duration/2)
// 	{
// 		x_desired = x_init_;
// 		rotation_desired = rotation_init_;
// 	}
// 	else
// 	{
// 		x_desired = target_x.head(3);
// 		rotation_desired << target_x.tail(9);
// 	}
// 	// ----------------------------------------------------------------------------

// 	// ----------------------------------------------------------------------------
// 	// Gain
// 	Matrix6d Kp, Kv;
// 	Vector6d Kp_diag, Kv_diag;
// 	Kp_diag << 50, 50, 50, 10, 10, 10;
// 	Kv_diag << 50, 50, 50, 10, 10, 10;
// 	Kp = Kp_diag.asDiagonal();
// 	Kv = Kv_diag.asDiagonal();

// 	// Calculate Control Force
// 	Vector6d control_force;
// 	x_error.segment<3>(0) = x_desired - x_;	
// 	x_error.segment<3>(3) = DyrosMath::getPhi(rotation_desired, rotation_);
// 	xd_error = xd_desired - x_dot_;
// 	control_force = Kp*x_error + Kv*xd_error;

// 	// Apply Torque
// 	torque_desired_ = j_.transpose() * control_force + g_;
// 	// ----------------------------------------------------------------------------

// 	// ----------------------------------------------------------------------------
// 	// Save data
// 	// [ desired position, current position, desired orientation, current orientation, joint position]
// 	recordHW6(11, duration, x_desired, rotation_desired);
// 	// ----------------------------------------------------------------------------
// }

// void ArmController::hw_6_1_2(const Vector12d &target_x, double duration)
// {
	
// 	Vector3d x_desired; // position
// 	Matrix3d rotation_desired; // orientation
// 	Vector6d xd_desired, x_error, xd_error; // v, w
	
// 	// Desired trajectory
// 	// ----------------------------------------------------------------------------
// 	// desired linear velocity
// 	for (int i = 0; i < 3; i++) 
// 	{
// 		xd_desired(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
// 			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
// 	}

// 	// Get target rotation matrix
// 	Matrix3d rotation;
// 	for (int i = 0; i < 3; i++) 
// 	{
// 		rotation.block<3, 1>(0, i) = target_x.segment<3>(3 + i * 3);
// 	}

// 	// desired angular velocity
// 	xd_desired.tail(3) = DyrosMath::rotationCubicDot(play_time_, control_start_time_,
// 		control_start_time_ + duration, Vector3d::Zero(), Vector3d::Zero(), rotation_init_, rotation);

// 	// desired position
// 	for (int i = 0; i < 3; i++)
// 	{
// 		x_desired(i) = DyrosMath::cubic(play_time_, control_start_time_,
// 			control_start_time_ + duration, x_init_(i), target_x(i), 0, 0);
// 	}

// 	// desired orientation
// 	Matrix3d rotation_desired = DyrosMath::rotationCubic(play_time_, control_start_time_,
// 		control_start_time_ + duration, rotation_init_, rotation);
// 	// ----------------------------------------------------------------------------
	
// 	// ----------------------------------------------------------------------------
// 	// Gain
// 	Matrix6d Kp, Kv;
// 	Vector6d Kp_diag, Kv_diag;
// 	Kp_diag << 50, 50, 50, 10, 10, 10;
// 	Kv_diag << 50, 50, 50, 10, 10, 10;
// 	Kp = Kp_diag.asDiagonal();
// 	Kv = Kv_diag.asDiagonal();

// 	// Calculate Control Force
// 	Vector6d control_force;
// 	x_error.segment<3>(0) = x_desired - x_;	
// 	x_error.segment<3>(3) = DyrosMath::getPhi(rotation_desired, rotation_);
// 	xd_error = xd_desired - x_dot_;
// 	control_force = Kp*x_error + Kv*xd_error;

// 	// Apply Torque
// 	torque_desired_ = j_.transpose() * control_force + g_;
// 	// ----------------------------------------------------------------------------

// 	// ----------------------------------------------------------------------------
// 	// Save data
// 	// [ desired position, current position, desired orientation, current orientation, joint position]
// 	recordHW6(12, duration, x_desired, rotation_desired);
// 	// ----------------------------------------------------------------------------
// }


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

