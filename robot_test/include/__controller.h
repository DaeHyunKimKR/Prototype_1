#pragma once // ?
#ifndef __CONTROLLER_H //?
#define __CONTROLLER_H //?
#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;


class CController
{
public:
	CController();
	virtual ~CController(); //	

	void inverseKin(double x, double y, double alpha); // inverse Kinematics
	void forwardKin(double q1, double q2, double q3); //forward kinematics
	void Finite_State_Machine(int button, float axes1, float axes2, double time);
	void get_present_position(int32_t* dxl_present_position);
	void get_linear_present_position(int linear_present_position);
	//void SetPosition(double target[], double time, double duration);
	// void SetTorque(double target[], double time, double duration);
	//void Finite_State_Machine(double time); *******되돌리기
	double CNT2RAD(double joint_num, double cnt);
	double M2CNT(double joint_num, double m);

	bool _mode, _mode2;
	int _goal_position[5];
	int _dxl_present_position[5];
	int _x_goal[5];
	

public:
	double _q[5],_z,_x,_y, _gamma_1, _k1_1, _k2_1, _x_e, _y_e, _z_e, _alpha_e, FK_q1, FK_q2, FK_q3, IK_q1, IK_q2, IK_q3, IK_x, IK_y, IK_a, IK_z;
	int _linear_goal_position;
	int _new_mode;

private:
	double _cos_q2, _sin_q2_1, _sin_q2_2, _q2_1, _q2_2, _k1_2, _k2_2, _gamma_2, _q1_1, _q1_2, _q3_1, _q3_2;
	
	
	// VectorXd _qdes; //desired joint angle vector
	// VectorXd _qdotdes; //desired joint velocity vector
	int getValue();
	int getValue2();
	int DEG2CNT(double joint_num, double deg);
	double DEG2RAD(double joint_num, double deg);
	double RAD2DEG(double joint_num, double rad);
	
	double RANGE(double angle);
	double _length_link1 ;
	double _length_link2 ;
	double _length_link3 ;
	bool _rev;

	double _now_time4;
	int _box, _table;
	int _cnt;

	typedef enum{
		Init_Pose_Task, // move to initial pose and wait
		Pick_and_Load_Snack,  // grasp item in the robot and move to target position, than land the item on the target position, and move back to initial posture
		FK_Task,
		No_Task
	} Task;
	typedef enum{
		Ready_State, // keep pose and wait for button command
		FK_State,
		Initial_State, // move to initial pose
		Box_Lane, // move to Box Lane pose
		Drop_Point, // move to Drop Point pose
		Grab_Box, // move to Grab Box pose
		Gripper_Open_State, // open gripper
		Gripper_Close_State // close gripper
	} State;

	State _CurrentState;
	State _PreviousState;
	Task _CurrentTask;
	double _init_time;
	double _operation_time;
	int _x_ready[5];
	int _x_init[5];
	int _x_Box_Lane[5];
	int _x_Grab_Box[5];


};

#endif