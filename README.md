# Panda Control in V-REP (CoppeliaSim)

## This code is made by Suhan Park (https://github.com/psh117/panda_control_vrep_linux)

## Requirements
- CoppeliaSim 4.0.0
- CMake
- RBDL (https://github.com/ORB-HD/rbdl-orb)

## Installation and Execution
```sh
git clone --recursive https://github.com/psh117/panda_control_vrep_linux
cd panda_control_vrep_linux
mkdir build
cd build
cmake ..
make
```
- make sure you've enabled shared memory control setting in v-rep (coppeliasim). 
the port is set to -3
( remoteApiConnections.txt -> portIndex1_port = -3 )
- make sure v-rep (coppeliasim) scene is loaded and ready to move before execution
```sh
./panda_control_vrep
````

## Control Mode
There are two control mode :
- Position Mode (HW2 - HW3)
- Torque Mode (HW4 - HW7)

If you use position mode, you have to edit some codes : 
1. main.cpp 
```
// VRepBridge vb(VRepBridge::CTRL_TORQUE); // Torque controlled
VRepBridge vb(VRepBridge::CTRL_POSITION); // Position controlled 
const double hz = 100;  // For Position control, 100hz is appropriate, else 1000hz
  
  ...
  
case '1':
	ac.setMode("hw_2_1");
	break;
case '2':
	ac.setMode("hw_2_2");
	break;
case '3':
	ac.setMode("hw_2_3");
	break;
case '4':
	ac.setMode("hw_3_1");
	break;	
case '5':
	ac.setMode("hw_3_2");
	break;		
case '6':
	ac.setMode("hw_3_3");
	break;			
// case '1':
// 	ac.setMode("hw_4_1");
// 	break;
// case '2':
// 	ac.setMode("hw_4_2");
// 	break;
// case '3':
// 	ac.setMode("hw_4_3");
// 	break;
// case '4':
// 	ac.setMode("hw_5_1");
// 	break;
// case '5':
// 	ac.setMode("hw_5_2");
// 	break;
// case '6':
// 	ac.setMode("hw_7");
// 	break;
```
2. controller.cpp
```
// moveJointPositionTorque(target_position, 1.0);
moveJointPosition(target_position, 1.0);
```
3. use 'franka_panda_position.ttt'

else if you want torque mode :
1. main.cpp 
```
VRepBridge vb(VRepBridge::CTRL_TORQUE); // Torque controlled
// VRepBridge vb(VRepBridge::CTRL_POSITION); // Position controlled 
const double hz = 1000;  // For Position control, 100hz is appropriate, else 1000hz
  
  ...
  
// case '1':
// 	ac.setMode("hw_2_1");
// 	break;
// case '2':
// 	ac.setMode("hw_2_2");
// 	break;
// case '3':
// 	ac.setMode("hw_2_3");
// 	break;
// case '4':
// 	ac.setMode("hw_3_1");
// 	break;	
// case '5':
// 	ac.setMode("hw_3_2");
// 	break;		
// case '6':
// 	ac.setMode("hw_3_3");
// 	break;			
case '1':
	ac.setMode("hw_4_1");
	break;
case '2':
	ac.setMode("hw_4_2");
	break;
case '3':
	ac.setMode("hw_4_3");
	break;
case '4':
	ac.setMode("hw_5_1");
	break;
case '5':
	ac.setMode("hw_5_2");
	break;
case '6':
	ac.setMode("hw_7");
```
2. controller.cpp
```
moveJointPositionTorque(target_position, 1.0);
// moveJointPosition(target_position, 1.0);
```
3. use 'franka_panda_wo_torque.ttt'

## If you don't have "Vortex" engine, change physics engine to "Bullet 2.78"
