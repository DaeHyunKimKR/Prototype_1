#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <wheel.h>
#include <tray.h>
#include <unified_controller.h>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include "scara_control/Scara_status.h"
#include "scara_control/Scara_cmd.h"
#include "linear_control/linear_status.h"
#include "linear_control/linear_cmd.h"
#include <robot_test/robot_data.h>

#define DEG2RAD 0.017453289
#define PERIOD_NS 100000000 // 10Hz
#define SEC_IN_NSEC 1000000000

using namespace std;

double time_tmp , now_time;

Cwheel wheel;
CUController robot;
CScara front_arm;
CScara back_arm;
Ctray tray_front(3);
Ctray tray_back(1);

int old_x_goal[5];
bool test_state = 1;

//for mobile base move
double linear_vel = 0.0;
double angular_vel = 0.0;

//for joystick use
bool mode = 0; 
float axes[8];
int button[8];

//for error check
float error_code_forward;

//for emergency stop
bool servo_ctrl = false;

//for scara robot
double jointangle_scara_front[3]; //1,2,3
double jointangle_scara_back[3]; //1,2,3
double jointangle_cmd_scara_front[3]; //1,2,3
double jointangle_cmd_scara_back[3]; //1,2,3
double position_cmd_scara_front[3]; //1,2,3
double position_cmd_scara_back[3]; //1,2,3
bool bool_joint_cmd;
double _ee_pos[3];

//for linear motors
double position_linear[2];
double position_cmd_linear[2];

//for tray
//double sensor_tray_front[2];
bool _bool_tray_front_motion = false;
double _tray_front_motion_time = 0.0;
double _tray_front_init_time = 0.0;
bool _bool_tray_back_motion = false;
double _tray_back_motion_time = 0.0;
double _tray_back_init_time = 0.0;

bool _tmp_bool = false;


//for data msgs

string state__[9] = {"Ready", "Front_Table_Lane", "Back_Table_Lane", "Front_Table", "Back_Table", "Item_Move", "Drop_Point", "Drop_Lane", "IK_State"};



void FKforPos(double jointangle[])
{
    
  _ee_pos[0] = 0.35 * cos(jointangle[0]*DEG2RAD) + 0.35 * cos(jointangle[0]*DEG2RAD + jointangle[1]*DEG2RAD) + 0.2 * cos(jointangle[0]*DEG2RAD + jointangle[1]*DEG2RAD + jointangle[2]*DEG2RAD); // m
	_ee_pos[1] = 0.35 * sin(jointangle[0]*DEG2RAD) + 0.35 * sin(jointangle[0]*DEG2RAD + jointangle[1]*DEG2RAD) + 0.2 * sin(jointangle[0]*DEG2RAD + jointangle[1]*DEG2RAD + jointangle[2]*DEG2RAD); // m
	_ee_pos[2] = jointangle[0]*DEG2RAD + jointangle[1]*DEG2RAD + jointangle[2]*DEG2RAD; // rad
}

void JoystickCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  for(int i = 0 ; i < 8 ; i++)
  {
    axes[i] = joy_msg->axes[i];
  }
    for(int i = 0 ; i < 8 ; i++)
  {
    button[i] = joy_msg->buttons[i];
  }
  if (button[0] == 1)
  {
    if(mode == 1)
    {
      mode = 0;
    }
    else
    {
      mode = 1;
    }
  }
}

void velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  linear_vel = vel->linear.x;
  angular_vel = vel->angular.z;
}

void taskCallback(const std_msgs::Float32MultiArray::ConstPtr& task_cmd)
{
  robot._task_tray_cmd = task_cmd->data[0];// tray number 
  robot._target_position_from_vision[0] = task_cmd->data[1];
  robot._target_position_from_vision[1] = task_cmd->data[2];
  robot._target_position_from_vision[2] = task_cmd->data[3];
  // cout << robot._task_tray_cmd << " " << robot._target_position_from_vision[0] << " " << robot._target_position_from_vision[1] << " " << robot._target_position_from_vision[2] << '\n';
}

void scaraFrontCallback(const scara_control::Scara_status::ConstPtr& scara_state)
{
  jointangle_scara_front[0] = scara_state->current_joint[0];
  jointangle_scara_front[1] = scara_state->current_joint[1];
  jointangle_scara_front[2] = scara_state->current_joint[2];
}
void scaraBackCallback(const scara_control::Scara_status::ConstPtr& scara_state)
{
  jointangle_scara_back[0] = scara_state->current_joint[0];
  jointangle_scara_back[1] = scara_state->current_joint[1];
  jointangle_scara_back[2] = scara_state->current_joint[2];
}

void linearCallback(const linear_control::linear_status::ConstPtr& linear_state)
{
  position_linear[0] = linear_state->current_position[0];
  position_linear[1] = linear_state->current_position[1];
}

void servoCallback(const std_msgs::Bool::ConstPtr& servo_msg)
{
  servo_ctrl = servo_msg->data;
}

// void dataCallback(const std_msgs::Float64MultiArray::ConstPtr& data_msgs)
// {
//   robot._CurrentState = data_msgs->data[0];
//   robot._q_goal_forward[0] = data_msgs->data[1];
//   robot._q_goal_forward[1] = data_msgs->data[2];
//   robot._q_goal_forward[2] = data_msgs->data[3];
//   robot._height_goal_linear = data_msgs->data[4];
//   //cout << Control._task_tray_cmd << " " << Control._target_position_from_vision[0] << " " << Control._target_position_from_vision[1] << " " << Control._target_position_from_vision[2] << '\n';
// }

// void trayFrontCallback(const std_msgs::UInt8MultiArray::ConstPtr& tray_sensor_state)
// {
//   sensor_tray_front[0] = tray_sensor_state->data[0];
//   sensor_tray_front[1] = tray_sensor_state->data[1];
// }



void initialize()
{
  for (int i=0; i<8; i++)
  {
    axes[i] = 0.0;
    button[i] = 0;
  }

  for (int i=0; i<5; i++)
  {
    old_x_goal[i] = 0;
  }

  for (int i=0; i<3; i++)
  {
    jointangle_scara_front[i] = 0.0;
    jointangle_scara_back[i] = 0.0;
    jointangle_cmd_scara_front[i] = 0.0;
    jointangle_cmd_scara_back[i] = 0.0;
    position_cmd_scara_front[i] = 0.0;
    position_cmd_scara_back[i] = 0.0;
    bool_joint_cmd = true;
  }

  for (int i=0; i<2; i++)
  {
    position_linear[i] = 0.0;
    position_cmd_linear[i] = 0.0;
    //sensor_tray_front[i] = 0.0;
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");
  struct timespec ts;
  struct timespec ts2;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  clock_gettime(CLOCK_MONOTONIC, &ts2);
  int start_time = ts2.tv_sec;



  ros::NodeHandle n_;
  ros::NodeHandle n_scara_front;
  ros::NodeHandle n_scara_back;
  ros::NodeHandle n_linear;
  ros::NodeHandle n_tray_front_pub;
  ros::NodeHandle nh_data_msgs;

  ros::Publisher pub_ = n_.advertise<std_msgs::Float32MultiArray>("velocity", 1000);
  ros::Publisher scara_front_pub = n_scara_front.advertise<scara_control::Scara_cmd>("/scara_control1", 1000);
  ros::Publisher scara_back_pub = n_scara_back.advertise<scara_control::Scara_cmd>("/scara_control2", 1000);
  ros::Publisher linear_pub = n_linear.advertise<linear_control::linear_cmd>("/linear_control", 100);
  ros::Publisher tray_front_pub = n_tray_front_pub.advertise<std_msgs::Float64MultiArray>("motorCommand", 100);
  ros::Subscriber sub_taskcmd_ = n_scara_front.subscribe("/arm/cmd",1,taskCallback);
  ros::Publisher data_pub = nh_data_msgs.advertise<robot_test::robot_data>("/arm_info", 1);
  ros::Publisher pub_taskstate_ = n_.advertise<std_msgs::Int32>("/arm_status", 1000);
  //pub_taskstate_ = n_.advertise<std_msgs::Int32>("task_state", 1000); 



  ros::NodeHandle nh_wheels;
  ros::NodeHandle nh_joy;
  //ros::NodeHandle nh_scara;
  ros::NodeHandle nh_scara_front;
  ros::NodeHandle nh_scara_back;
  ros::NodeHandle nh_linear;
  ros::NodeHandle nh_task_cmd;
  ros::NodeHandle nh_servo;
  //ros::NodeHandle nh_tray_front;
  
  ros::Subscriber sub = nh_wheels.subscribe("cmd_vel",10,velCallback);
  ros::Subscriber sub2 = nh_joy.subscribe("joy", 10, JoystickCallback);
  //ros::Subscriber sub3 = nh_scara.subscribe("/scara_status1",10,scaraFrontCallback);
  ros::Subscriber sub3 = nh_scara_front.subscribe("/scara_status1",10,scaraFrontCallback);
  ros::Subscriber sub4 = nh_scara_back.subscribe("/scara_status2",10,scaraBackCallback);
  ros::Subscriber sub5 = nh_linear.subscribe("/linear_status",10,linearCallback);
  ros::Subscriber sub6 = nh_servo.subscribe("/stop/request",1,servoCallback);
  //ros::Subscriber sub7 = nh_tray_front.subscribe("sensorStatus",10,trayFrontCallback);
  


  std_msgs::Float32MultiArray velocity;
  scara_control::Scara_cmd scara_forward_msgs;
  scara_control::Scara_cmd scara_backward_msgs;
  linear_control::linear_cmd linear_msgs;
  std_msgs::Float64MultiArray data_msgs;
  //std_msgs::Float64MultiArray tray_front_msgs;






  initialize(); //initialize variables 

  //////////////// initialize communication
  wheel.Open_port();
  tray_front.Open_port();
  tray_back.Open_port();
  
  cout << "Start RS485 Communication!"<< endl << "Wait for 6 sec" << endl<<endl;

  //servo on scara
  scara_forward_msgs.cmd = "on";
  scara_forward_msgs.node.clear();
  scara_forward_msgs.node.push_back(1);
  scara_forward_msgs.node.push_back(2);
  scara_forward_msgs.node.push_back(3);  
  scara_front_pub.publish(scara_forward_msgs);
  scara_backward_msgs.cmd = "on";
  scara_backward_msgs.node.clear();
  scara_backward_msgs.node.push_back(4);
  scara_backward_msgs.node.push_back(5);
  scara_backward_msgs.node.push_back(6);
  scara_back_pub.publish(scara_backward_msgs);

  //servo on linear
  linear_msgs.cmd = "on";
  linear_msgs.node.clear();
  linear_msgs.node.push_back(1);
  linear_msgs.node.push_back(2);
  linear_pub.publish(linear_msgs);
  

  cout << "Robot is Ready to Move!" << endl<<endl;

  //////////////// Control loop
  while(ros::ok())
  {
    while(ts.tv_nsec >= SEC_IN_NSEC)
    {
        ts.tv_sec++;
        ts.tv_nsec -= SEC_IN_NSEC;
    }

    while(ts2.tv_nsec >= SEC_IN_NSEC)
    {
        ts2.tv_sec++;
        ts2.tv_nsec -= SEC_IN_NSEC;
    }

    ts.tv_nsec +=PERIOD_NS;
    clock_gettime(CLOCK_MONOTONIC, &ts2);
    time_tmp = ts2.tv_sec - start_time;
    now_time = time_tmp + ts2.tv_nsec / 1000000000.0;


  // Error check
    if( jointangle_scara_front[0] > -91 && jointangle_scara_front[0] < 91 )
    {
    }
    else if( jointangle_scara_front[0] < -91 || jointangle_scara_front[0] > 91 )
    {
      error_code_forward = 1112;
    }
    else
    {
      error_code_forward = 1111;
    }

    if( jointangle_scara_front[1] > -91 && jointangle_scara_front[1] < 91 )
    {
    }
    else if( jointangle_scara_front[1] < -91 || jointangle_scara_front[1] > 91 )
    {
      error_code_forward = 1122;
    }
    else
    {
      error_code_forward = 1121;
    }

    if( jointangle_scara_front[2] > -91 && jointangle_scara_front[2] < 91 )
    {
    }
    else if( jointangle_scara_front[2] < -91 || jointangle_scara_front[2] > 91 )
    {
      error_code_forward = 1132;
    }
    else
    {
      error_code_forward = 1131;
    }
    
  // Emergency Stop
  /*
    if(servo_ctrl == false)
    {
      scara_forward_msgs.cmd = "on";
      scara_forward_msgs.node.clear();
      scara_forward_msgs.node.push_back(1);
      scara_forward_msgs.node.push_back(2);
      scara_forward_msgs.node.push_back(3);  
      scara_front_pub.publish(scara_forward_msgs);
      scara_backward_msgs.cmd = "on";                                                                                                                               
      scara_backward_msgs.node.clear();
      scara_backward_msgs.node.push_back(4);
      scara_backward_msgs.node.push_back(5);
      scara_backward_msgs.node.push_back(6);
      scara_back_pub.publish(scara_backward_msgs);
      linear_msgs.cmd = "on";
      linear_msgs.node.clear();
      linear_msgs.node.push_back(1);
      linear_msgs.node.push_back(2);
      linear_pub.publish(linear_msgs);

      wheel.Servo_on();
      tray_front.Control_on();
    }
    else if(servo_ctrl == true)
    {
      scara_forward_msgs.cmd = "off";
      scara_forward_msgs.node.clear();
      scara_forward_msgs.node.push_back(1);
      scara_forward_msgs.node.push_back(2);
      scara_forward_msgs.node.push_back(3);  
      scara_front_pub.publish(scara_forward_msgs);
      scara_backward_msgs.cmd = "off";                                                                                                                               
      scara_backward_msgs.node.clear();
      scara_backward_msgs.node.push_back(4);
      scara_backward_msgs.node.push_back(5);
      scara_backward_msgs.node.push_back(6);
      scara_back_pub.publish(scara_backward_msgs);
      linear_msgs.cmd = "off";
      linear_msgs.node.clear();
      linear_msgs.node.push_back(1);
      linear_msgs.node.push_back(2);
      linear_pub.publish(linear_msgs);

      wheel.Servo_off();
      tray_front.Control_off();
    }
  */

  // for wheel control
	  if(mode == 0)
	  {
	    wheel.JoyStick_msg(axes,button);
	  }
	  else if(mode == 1)
	  {
	    wheel.velocity_target(linear_vel,angular_vel);
	  }
    
    velocity.data.clear();
	  velocity.data.push_back(wheel._LeftVelocity_MS);
	  velocity.data.push_back(wheel._RightVelocity_MS);
	  velocity.data.push_back(wheel._LeftPosition);
	  velocity.data.push_back(wheel._RightPosition);

    //for robot arm control        
    robot.Scara_forward.read_state(jointangle_scara_front); //
    robot.Scara_backward.read_state(jointangle_scara_back); //

    robot.Linear_forward.read_state(position_linear[0]); //
    robot.Linear_backward.read_state(position_linear[1]); //


    //robot._task_tray_cmd = 3;

    robot.Finite_State_Machine(button[5], axes[2],  axes[5], now_time);
    
    if(robot._control_arm_num == 1)
    {
        if(robot.Scara_forward._bool_ee_control == false) // joint command from high level controller
        {
          scara_forward_msgs.cmd = "joint";
          scara_forward_msgs.node.clear();
          scara_forward_msgs.node.push_back(1);
          scara_forward_msgs.node.push_back(2);
          scara_forward_msgs.node.push_back(3);
          scara_forward_msgs.angle1 = robot.Scara_forward._q_goal[0];
          scara_forward_msgs.angle2 = robot.Scara_forward._q_goal[1];
          scara_forward_msgs.angle3 = robot.Scara_forward._q_goal[2];
          scara_forward_msgs.time = robot.Scara_forward._time_motion;
        }
        else//end-effector position command from high level controller (in local coordinate)
        {
            //TODO
        }
    }
    else if(robot._control_arm_num == 2)
    {
        if(robot.Scara_backward._bool_ee_control == false) // joint command from high level controller
        {
          scara_backward_msgs.cmd = "joint";
          scara_backward_msgs.node.clear();
          scara_backward_msgs.node.push_back(4);
          scara_backward_msgs.node.push_back(5);
          scara_backward_msgs.node.push_back(6);
          scara_backward_msgs.angle1 = robot.Scara_backward._q_goal[0];
          scara_backward_msgs.angle2 = robot.Scara_backward._q_goal[1];
          scara_backward_msgs.angle3 = robot.Scara_backward._q_goal[2];
          scara_backward_msgs.time = robot.Scara_forward._time_motion;
        }
        else //end-effector position command from high level controller (in local coordinate)
        {
            //TODO
        }
    }
    
    else if(robot._control_arm_num == 3)
    {
        if((robot.Scara_forward._bool_ee_control && robot.Scara_backward._bool_ee_control) == false) // joint command from high level controller
        {
          scara_forward_msgs.cmd = "joint";
          scara_forward_msgs.node.clear();
          scara_forward_msgs.node.push_back(1);
          scara_forward_msgs.node.push_back(2);
          scara_forward_msgs.node.push_back(3);
          scara_forward_msgs.angle1 = robot.Scara_forward._q_goal[0];
          scara_forward_msgs.angle2 = robot.Scara_forward._q_goal[1];
          scara_forward_msgs.angle3 = robot.Scara_forward._q_goal[2];
          scara_forward_msgs.time = robot.Scara_forward._time_motion;

          scara_backward_msgs.cmd = "joint";
          scara_backward_msgs.node.clear();
          scara_backward_msgs.node.push_back(4);
          scara_backward_msgs.node.push_back(5);
          scara_backward_msgs.node.push_back(6);
          scara_backward_msgs.angle1 = robot.Scara_backward._q_goal[0];
          scara_backward_msgs.angle2 = robot.Scara_backward._q_goal[1];
          scara_backward_msgs.angle3 = robot.Scara_backward._q_goal[2];
          scara_backward_msgs.time = robot.Scara_forward._time_motion;
        }
        else //end-effector position command from high level controller (in local coordinate)
        {
            //TODO
        }
    }
    
    else if(robot._control_arm_num == 0)
    {
        scara_forward_msgs.cmd = "status";
        scara_forward_msgs.node.clear();
        scara_forward_msgs.node.push_back(1);
        scara_forward_msgs.node.push_back(2);
        scara_forward_msgs.node.push_back(3);
        scara_forward_msgs.node.push_back(4);
        scara_forward_msgs.node.push_back(5);
        scara_forward_msgs.node.push_back(6);

        scara_backward_msgs.cmd = "status";
        scara_backward_msgs.node.clear();
        scara_backward_msgs.node.push_back(1);
        scara_backward_msgs.node.push_back(2);
        scara_backward_msgs.node.push_back(3);
        scara_backward_msgs.node.push_back(4);
        scara_backward_msgs.node.push_back(5);
        scara_backward_msgs.node.push_back(6);
    }    

    linear_msgs.cmd = "joint";
    linear_msgs.node.clear();
    linear_msgs.node.push_back(1);
    linear_msgs.node.push_back(2);
    linear_msgs.position1 = -robot.Linear_forward._height_goal;
    linear_msgs.position2 = -robot.Linear_backward._height_goal;
    linear_msgs.time = robot.Linear_forward._time_motion;
    linear_msgs.time = robot.Linear_backward._time_motion;

    // linear_msgs.cmd = "status";
    // linear_msgs.node.clear();
    // linear_msgs.node.push_back(1);
    // linear_msgs.node.push_back(2);

    if(robot.tray_forward._bool_tray_cmd == true)
    {
        if(robot.tray_forward._velocity_cmd > 0.001)
        {
          // tray_front.Sensor_Based(1);
          tray_front.Velocity_Command(1);
        }
        else if(robot.tray_forward._velocity_cmd < -0.001)
        {
          tray_front.Velocity_Command(2);
        }
    }
    else
    {
      tray_front.Velocity_Command(0);
    }

    if(robot.tray_backward._bool_tray_cmd == true)
    {
        if(robot.tray_backward._velocity_cmd > 0.001)
        {
          tray_back.Velocity_Command(1);
        }
        else if(robot.tray_backward._velocity_cmd < -0.001)
        {
          tray_back.Velocity_Command(2);
        }
    }
    else
    {
      tray_back.Velocity_Command(0);
    }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////// DATA MESSAGES /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    robot_test::robot_data robot_data;
    
    robot_data.robot_state = state__[robot._CurrentState];
    // cout << robot._CurrentState << endl<<endl;

    // robot_data.joint1 = robot._q_goal_forward[0];
    // robot_data.joint2 = robot._q_goal_forward[1];
    // robot_data.joint3 = robot._q_goal_forward[2];

    robot_data.joint1 = jointangle_scara_front[0];
    robot_data.joint2 = jointangle_scara_front[1];
    robot_data.joint3 = jointangle_scara_front[2];
    

    robot_data.height = position_linear[0];
    robot_data.tray = tray_front.tray_status;
    //scara_front.update_forward_kinematics();
    // robot_data.front_pos_x = robot._ee_pos[0];
    // robot_data.front_pos_y = robot._ee_pos[1];
    // robot_data.front_pos_a = robot._ee_pos[2];

    FKforPos(jointangle_scara_front);
    robot_data.front_pos_x = _ee_pos[0];
    robot_data.front_pos_y = _ee_pos[1];
    robot_data.front_pos_z = position_linear[0];
    robot_data.front_pos_a = _ee_pos[2];

    robot_data.table_status_1 = robot._table_status[0];
    robot_data.table_status_2 = robot._table_status[1];
    robot_data.table_status_3 = robot._table_status[2];
    robot_data.table_status_4 = robot._table_status[3];
    robot_data.table_status_5 = robot._table_status[4];
    robot_data.table_status_6 = robot._table_status[5];
    robot_data.table_status_7 = robot._table_status[6];
    robot_data.table_status_8 = robot._table_status[7];
    robot_data.table_status_9 = robot._table_status[8];
    robot_data.table_status_10 = robot._table_status[9];
    robot_data.table_status_11 = robot._table_status[10];
    robot_data.table_status_12 = robot._table_status[11];
    robot_data.table_status_13 = robot._table_status[12];
    robot_data.table_status_14 = robot._table_status[13];
    robot_data.table_status_15 = robot._table_status[14];
    robot_data.table_status_16 = robot._table_status[15];

    // robot_data.Error_Code = error_code_forward;
    

    data_pub.publish(robot_data);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




    //publish ros msgs
    
    pub_.publish(velocity);
    //scara_pub.publish(scara_msgs);
    scara_front_pub.publish(scara_forward_msgs);
    scara_back_pub.publish(scara_backward_msgs);
    linear_pub.publish(linear_msgs);

    //tray_front_pub.publish(tray_front_msgs);

    std_msgs::Int32 task_state;
    task_state.data = robot._task_state;
    pub_taskstate_.publish(task_state);
    

    //cout << position_linear[0] << " " << position_linear[1] << endl;    
    //cout << jointangle_scara_front[0] << " " << jointangle_scara_front[1] << " " << jointangle_scara_front[2] << " " << position_linear[0]  << endl;
    //cout << jointangle_scara_back[0] << " " << jointangle_scara_back[1] << " " << jointangle_scara_back[2] << endl <<endl;
    
    
    // cout << "\033[2J\033[1;1H";

    ros::spinOnce();

  }
  
  return (0);
}


