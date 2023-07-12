#include <iostream>
#include <string>
#include "vrep_bridge.h"

#include "controller.h"

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;
 
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
	ungetc(ch, stdin);
	return 1;
	}

	return 0;
}


int main()
{
	// VRepBridge vb(VRepBridge::CTRL_TORQUE); // Torque controlled
	VRepBridge vb(VRepBridge::CTRL_POSITION); // Position controlled 
	const double hz = 100;  // For Position control, 100hz is appropriate, else 1000hz
	ArmController ac(hz);
	bool is_simulation_run = true;
	bool exit_flag = false;
	bool is_first = true;

	while (vb.simConnectionCheck() && !exit_flag)
	{
		vb.read();
		ac.readData(vb.getPosition(), vb.getVelocity(), vb.getGripperPosition(), vb.getGripperVelocity(), vb.getGripperForce(), vb.getFTData());
		if (is_first)
		{
			vb.simLoop();
			vb.read();
			ac.readData(vb.getPosition(), vb.getVelocity(), vb.getGripperPosition(), vb.getGripperVelocity(), vb.getGripperForce(), vb.getFTData());
			cout << "Initial q: " << vb.getPosition().transpose() << endl;
			is_first = false;
			ac.initPosition();
		}

		if (kbhit())
		{
			int key = getchar();
			switch (key)
			{
				// Implement with user input
			case 'i':
				ac.setMode("joint_ctrl_init");
				break;
			case 'o':
				ac.setMode("gripper_open");
				break;
			case 'c':
				ac.setMode("gripper_close");
				break;
			case 'p':
				ac.setMode("pick_obstacle");
				break;
			case '1':
				ac.setMode("circle_traj");
				break;
			case '2':
				ac.setMode("square_traj");
				break;
			case '3':
				ac.setMode("eight_traj");
				break;
			case '\t':
				if (is_simulation_run) {
					cout << "Simulation Pause" << endl;
					is_simulation_run = false;
				}
				else {
					cout << "Simulation Run" << endl;
					is_simulation_run = true;
				}
				break;
			case 'q':
				is_simulation_run = false;
				exit_flag = true;
				break;
			default:
				break;
			}
		}

		if (is_simulation_run) {
			ac.compute();
			vb.setDesiredPosition(ac.getDesiredPosition());
			vb.setDesiredTorque(ac.getDesiredTorque());
			vb.setGripperDesiredPosition(ac.getDesiredGripperPosition());
			// vb.setGripperDesiredVelocity(ac.getDesiredGripperVelocity());
			// vb.setGripperDesiredForce(ac.getDesiredGripperForce());
		
			vb.write();
			vb.simLoop();
		}
	}
		
	return 0;
}
