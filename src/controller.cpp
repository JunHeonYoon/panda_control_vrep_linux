#include "controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <filesystem>

enum Traj_Type
{
	CIRCLE,
	SQUARE,
	EIGHT
};

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
	// q_		: joint positionPositionbasic jacobian
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
		control_start_tick_ = tick_;

		q_init_ = q_;
		qdot_init_ = qdot_;
		q_error_sum_.setZero();
		gq_init_ = gq_;
		gqdot_init_ = gqdot_;
		gforce_init_ = gforce_;

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
		target_position << 0.0, 0.0, 0.0, -M_PI / 2., 0.0, M_PI / 2, M_PI/4;
		// target_position << 0.0, 0.0, 0.0, -M_PI / 6., 0.0, M_PI / 2, 0;
		// target_position << 0.0, -M_PI / 3, 0.0, -M_PI / 2., 0.0, M_PI / 6, 0;
		// moveJointPositionTorque(target_position, 1.0);     
		moveJointPosition(target_position, 1.0);         

	}
	else if(control_mode_ == "gripper_open")
	{ 
		Vector2d target_gripper_position;
		target_gripper_position << 0.04, 0.04;
		// moveGripperPositionForce(target_gripper_position, 1);
		moveGripperPosition(target_gripper_position, 1);

		// Vector2d target_gripper_velocity;
		// target_gripper_velocity << 0.02, 0.02;
		// setGripperVelocity(target_gripper_velocity, 3);
	}
	else if(control_mode_ == "gripper_close")
	{ 
		// Vector2d target_gripper_position;
		// target_gripper_position << 0.0, 0.0;
		// // moveGripperPositionForce(target_gripper_position, 1); 
		// moveGripperPosition(target_gripper_position, 1);  

		// Vector2d target_gripper_velocity;
		// target_gripper_velocity << -0.02, -0.02;
		// setGripperVelocity(target_gripper_velocity, 3);

		gq_desired_ = gq_ - 0.01*VectorXd::Ones(2);
	}
	else if(control_mode_ == "pick_obstacle")
	{
		Vector12d target_x;
		target_x << 0.5, 0.00, 0.142,
			 0.707, -0.707, -0.000,
			-0.707, -0.707, -0.000,
			-0.000,  0.000, -1.000;
		CLIK(target_x, 3.0);
	}
	else if(control_mode_ == "circle_traj")
	{
		CLIKwTraj(Traj_Type(CIRCLE));
	}
	else if(control_mode_ == "square_traj")
	{
		CLIKwTraj(Traj_Type(SQUARE));
	}
	else if(control_mode_ == "eight_traj")
	{
		CLIKwTraj(Traj_Type(EIGHT));
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

void ArmController::recordFT(int file_number, double duration)
{
	if (play_time_ < control_start_time_ + duration)
	{
		hw_plot_files_[file_number] << play_time_ - control_start_time_ << " " << tip_ft_(2) << endl;
	}
}

void ArmController::printState()
{
	// TODO: Modify this method to debug your code

	static int DBG_CNT = 0;
	if (DBG_CNT++ > hz_ / 50.)
	{
		DBG_CNT = 0;

		cout << "-------------------------------------------------------" << endl;
		cout << "time     : " << std::fixed << std::setprecision(3) << play_time_ << endl;
		cout << "q now    :\t";
		cout << std::fixed << std::setprecision(3) << q_.transpose() << endl;
		cout << "q desired:\t";
		cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << endl;
		cout << "x        :\t";
		cout << x_.transpose() << endl;
		cout << "R        :\t" << endl;
		cout << std::fixed << std::setprecision(3) << rotation_ << endl;
		cout << "gripper now    :\t";
		cout << std::fixed << std::setprecision(3) << gq_.transpose() << endl;
		cout << "gripper desired:\t";
		cout << std::fixed << std::setprecision(3) << gq_desired_.transpose() << endl;
		cout << "gripper force now    :\t";
		cout << std::fixed << std::setprecision(3) << gforce_.transpose() << endl;
		cout << "gripper force desired:\t";
		cout << std::fixed << std::setprecision(3) << gforce_desired_.transpose() << endl;
		cout << "FT data:\t";
		cout << std::fixed << std::setprecision(3) << tip_ft_.transpose() << endl;
		cout << "gripper P gain :" << std::endl;
		cout<< std::fixed << std::setprecision(3) << (gq_desired_(0)-gq_((0)))/gforce_(0) <<endl;
		cout<< std::fixed << std::setprecision(3) << (gq_desired_(1)-gq_((1)))/gforce_(1) <<endl;
		
		if(control_mode_ == "hw_3_1")
		{
			j_2_.block<3, DOF>(0, 0); 
			
			cout << "hw 3-1 jacobian:" << endl;
			cout << j_2_ << endl;

			Vector6d x;
			x << x_, x_2_;

			cout << "hw 3-1 x:\t" << x.transpose() << endl;
		}
		cout << "-------------------------------------------------------" << endl;
		
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

void ArmController::moveGripperPosition(const Vector2d & target_position, double duration)
{
	Vector2d zero_vector;
	zero_vector.setZero();
	gq_desired_ = DyrosMath::cubicVector<2>(play_time_,
		control_start_time_,
		control_start_time_ + duration, gq_init_, target_position, zero_vector, zero_vector);
}

void ArmController::setGripperForce(const Vector2d & target_force, double duration)
{
	if(play_time_ < control_start_time_ + duration) gforce_desired_ = target_force;
	else gforce_desired_.setZero();
}

void ArmController::setGripperVelocity(const Vector2d &target_velocity, double duration)
{
	// if(play_time_ < control_start_time_ + duration) gqdot_desired_ = target_velocity;
	// else gqdot_desired_.setZero();
	gqdot_desired_ = target_velocity;
	
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

void ArmController::moveGripperPositionForce(const Vector2d &target_position, double duration)
{
	gq_desired_ = target_position;
	Matrix2d kp, kv;
	Vector2d gq_cubic, gqdot_cubic;
	
	kp = Matrix2d::Identity() * 500.0;
	kv = Matrix2d::Identity() * 20.0;

	for (int i = 0; i < 2; i++)
	{
		gqdot_cubic(i) = DyrosMath::cubicDot(play_time_, control_start_time_,
			control_start_time_ + duration, gq_init_(i), target_position(i), 0, 0);
		gq_cubic(i) = DyrosMath::cubic(play_time_, control_start_time_,
			control_start_time_ + duration, gq_init_(i), target_position(i), 0, 0);
	}

	Matrix2d gripper_m;
	gripper_m = Matrix2d::Identity() * 0.02;
	gforce_desired_ = gripper_m * (kp*(gq_cubic - gq_) + kv*(gqdot_cubic - gqdot_));
	// for(size_t i =0;i<2;i++)
	// {
	// 	if (gforce_desired_(i) > 10) gforce_desired_(i) = 10;
	// 	else if(gforce_desired_(i) < -10) gforce_desired_(i) = -10;
	// }
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
	// kp_diag << 50, 50, 50, 10, 10, 10;
	kp_diag << 40, 40, 40, 45, 42, 45;
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
	record(2, duration);
	// ----------------------------------------------------------------------------
}

void ArmController::CLIKwTraj(const int traj_type)
{
	//------------------------------------------------------------------------
	// Firstly, calculate desired pose and velocity ( end-effector ) for current time
	// by given data ( target pose, duration ).
	// Initial pose is given, initial and final velocity are zero.

	unsigned long time_index = tick_ - control_start_tick_;
	if(time_index >= traj_[traj_type].rows()) time_index = traj_[traj_type].rows() - 1;

	Vector3d x_desired; // only for position
	Matrix3d rotation_desired; // only for orientation
	Vector6d xd_desired; // v, w

	xd_desired.setZero();
	x_desired.setZero();
	// desired position
	x_desired = traj_[traj_type].block(time_index, 0, 1, 3).transpose() + x_init_;

	// desired orientation
	rotation_desired = rotation_init_; // Fix orientation

	// desired linear velocity
	xd_desired.head(3) = traj_[traj_type].block(time_index, 3, 1, 3).transpose(); // No angular velocity
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
	kp_diag << 20, 20, 20, 5, 5, 5;
	// kp_diag << 40, 40, 40, 45, 42, 45;
	Matrix6d kp = kp_diag.asDiagonal();

	// desired joint velocity and joint position
	// Get jacobian from desired joint value, not current joint value
	Matrix<double, 6, 7> j_qd = jacobianFromqd(0);
	Vector7d qd_desired = j_qd.transpose() * (j_qd*j_qd.transpose()).inverse() * ( xd_desired + kp * x_error );
	q_desired_ = q_desired_ + qd_desired / hz_; 
	// ----------------------------------------------------------------------------

	// ----------------------------------------------------------------------------
	// Save data
	// [ time, FT(z-direction) ]
	recordFT(4 + traj_type, double(traj_[traj_type].rows() / hz_));
	// ----------------------------------------------------------------------------
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
	gq_desired_.setZero();
	gforce_desired_.setZero();

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
    // mass[0] = 1.0;
    // mass[1] = 1.0;
    // mass[2] = 1.0;
    // mass[3] = 1.0;
    // mass[4] = 1.0;
    // mass[5] = 1.0;
    // mass[6] = 1.0;
	mass[0] = 2.083;
    mass[1] = 2.103;
    mass[2] = 1.629;
    mass[3] = 1.187;
    mass[4] = 1.710;
    mass[5] = 0.7175;
    mass[6] = 0.1130;

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

void ArmController::readTrajFile()
{
	traj_.resize(NUM_TRAJ_FILE);
	std::string file_path = "../traj/";
	for(int file_idx = 0; file_idx < NUM_TRAJ_FILE; file_idx++)
	{
		FILE *traj_file = NULL;
		traj_file = fopen( (file_path + traj_file_names_[file_idx] + ".txt").c_str(), "r" );
		int traj_length = 0;
		char tmp;

		if (traj_file == NULL)
		{
			std::cout<<"There is no txt file named: "<< traj_file_names_[file_idx] <<". Please edit code."<<std::endl;
			break;
		}

		while (fscanf(traj_file, "%c", &tmp) != EOF)
		{
			if (tmp == '\n')
				traj_length++;
		}

		fseek(traj_file, 0L, SEEK_SET);

		traj_[file_idx].setZero(traj_length, 6);
		double tmp_time;

		for(int i = 0; i < traj_length; i++)
		{
			fscanf(traj_file, "%lf %lf %lf %lf %lf \n",
					&tmp_time,
					&traj_[file_idx](i,1),
					&traj_[file_idx](i,2),
					&traj_[file_idx](i,4),
					&traj_[file_idx](i,5));
		}
		fclose(traj_file);
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

void ArmController::readData(const Vector7d &position, const Vector7d &velocity, const Vector2d &gripper_position, const Vector2d &gripper_velocity, const Vector2d &gripper_force, const Vector6d &tip_ft)
{
	for (size_t i = 0; i < dof_; i++)
	{
		q_(i) = position(i);
		qdot_(i) = velocity(i);
		torque_(i) = 0;
	}
	for (size_t i = 0; i < 2; i++)
	{
		gq_(i) = gripper_position(i);
		gqdot_(i) = gripper_velocity(i);
		gforce_(i) = gripper_force(i);
	}
	gq_(1) *= 0.5;
	for (size_t i = 0; i < 6; i++)
	{
		tip_ft_(i) = tip_ft(i);
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

const Vector2d & ArmController::getDesiredGripperPosition()
{
	return gq_desired_;
}

const Vector2d & ArmController::getDesiredGripperVelocity()
{
	return gqdot_desired_;
}

const Vector2d & ArmController::getDesiredGripperForce()
{
	return gforce_desired_;
}

const Vector6d & ArmController::getFTSensorData()
{
	return tip_ft_;
}

void ArmController::initPosition()
{
    q_init_ = q_;
    q_desired_ = q_init_;
	gq_init_ = gq_;
	gqdot_init_ = gqdot_;
	gq_desired_ = gq_init_;
	gqdot_desired_ = gqdot_init_;
	gforce_init_ = gforce_;
	gforce_desired_ = gforce_init_;
}

// ----------------------------------------------------

