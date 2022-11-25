#include "unified_controller.h"
#include <math.h>
#include <stdio.h>

#define PI 3.141592
#define RAD2DEG 57.295791433
#define DEG2RAD 0.017453289


CUController::CUController() //calc global position, implement fsm
{
    _now_time4 = 0.0;
    _cnt = 0;
    _control_arm_num = 0;//0 = none, 1:forward arm, 2: backward arm

    _CurrentState = Ready;
    _PreviousState = Ready;
    _CurrentTask = No_Task;
    _init_time = 0.0;
    _operation_time = 0.0;
 
    for(int i=0; i<3; i++)
    {
        _q_goal_forward[i] = 0.0;
        _x_goal_forward[i] = 0.0;
    }
    _height_goal_linear = 0.0;
    _bool_ee_control = false;
    

    for(int i=0; i<16; i++)
    {
        _check_table[i] == false;
    }

    _task_state = 2;
	_task_tray_cmd = 0;
	_pre_tray_cmd = 0;
	_target_position_from_vision[0] = 0.0;
	_target_position_from_vision[1] = 0.0;
	_target_position_from_vision[2] = 0.0;


}

CUController::~CUController()
{
}

void CUController::Finite_State_Machine( int button, float axes1, float axes2, double time)
{   //std::cout <<time << std::endl;
    //std::cout << _CurrentState << std::endl;
    //std::cout<< button << axes1 << axes2 << std::endl;
    //std::cout << _new_mode << std::endl;
    
    //_control_arm_num = 1; //0 = none, 1:forward arm, 2: backward arm    
    //_control_arm_num = 2; //0 = none, 1:forward arm, 2: backward arm 
    _control_arm_num = 3; //0 = none, 1:forward arm, 2: backward arm    


    if(_CurrentState == Ready)
    {
		_q_goal_forward[0] = -80.0*DEG2RAD; //q0
		_q_goal_forward[1] = 160.0*DEG2RAD; //q1
		_q_goal_forward[2] = 10.0*DEG2RAD; //q2
        _q_goal_backward[0] = -80.0*DEG2RAD; //q0
		_q_goal_backward[1] = 160.0*DEG2RAD; //q1
		_q_goal_backward[2] = 10.0*DEG2RAD; //q2
		_height_goal_linear = 0.57; //z 
        _bool_ee_control = false; //joint control
        _operation_time = 6.0; 
        // read_state_deg(_q_goal_forward);
        
        for(int i=0; i<16; i++)
        {
            if(_check_table[i] == true)
                {
                    _table_status[i] = 0;
                }
            else if(_check_table[i] == false)
                {
                    _table_status[i] = 1;
                }
        }
        //if(_PreviousState == Drop_Lane)
        //{
//
        //    if(time < _init_time + _operation_time)
        //    {
        //        _task_state = 0;
        //        _PreviousState = Ready;
        //        //_init_time = time; 
        //    }
        //    else if(time >= _init_time + _operation_time)
        //    {
        //        _task_state = 2;
        //        _PreviousState = Ready;
        //        _init_time = time; 
        //    }
        //    
        //}

        if( button == 1)
        {
            _operation_time = 7.0;
        
		    if(time < _init_time + _operation_time)            
            {

                _q_goal_forward[0] = 0.0*DEG2RAD; //q0
		        _q_goal_forward[1] = 90.0*DEG2RAD; //q1
		        _q_goal_forward[2] = 45.0*DEG2RAD; //q2
                _height_goal_linear = 1.45 - 1.023;

                Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
                Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);

                _q_goal_backward[0] = 0.0*DEG2RAD; //q0
		        _q_goal_backward[1] = -90.0*DEG2RAD; //q1
		        _q_goal_backward[2] = -45.0*DEG2RAD; //q2
		        //_height_goal_linear = 0.37; //z
                _height_goal_linear = 1.45 - 1.023; 

                Scara_backward.write_cmd_from_FSM(_operation_time, _q_goal_backward, _x_goal_backward, _bool_ee_control);
                Linear_backward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
                // read_state_deg(_q_goal_forward);
            }

            else if(time >= _init_time + _operation_time)
            {
                _CurrentState = Ready;
                _init_time = time;          
            }       



            /*
            cout << "Test Motion 1" <<endl;
            _CurrentTask = Test_Task;
            _CurrentState = Back_Table_Lane;
            _init_time = time;
            _PreviousState = Ready;
            */
        }
        else if ( axes1 < 0 ) //왼쪽버튼 누를시 -> 왼쪽 선반으로 //
        {
             cout << "Test Motion 1" <<endl;
             _CurrentTask = Test_Task;
             //_CurrentState = Back_Table_Lane;
             _task_tray_cmd = 3;
             _init_time = time;      
             _PreviousState = Ready;
        }
        else if ( axes2 < 0 ) //오른쪽버튼 누를시 -> 오른쪽 선반으로 //
        {
            
            // boxlane_initial_desired(13);
            
                       
             cout << "Test Motion 2" <<endl;
             _CurrentTask = Test_Task;
             //_CurrentState = Back_Table_Lane;
             _task_tray_cmd = 4;
             _init_time = time;      
             _PreviousState = Ready;
            
            
            
            // _CurrentTask = IK_Task;
            // _CurrentState = IK_State;
            // _init_time = time;      
            // _PreviousState = Ready;

            
        }
        
        else 
        {
            _CurrentTask = No_Task;
            _CurrentState = Ready;
            _PreviousState = Ready;
        }

        
        if(time < 10.0)
		{
			_task_tray_cmd = 0;
		}

		//if(_task_tray_cmd == 0 || _task_tray_cmd == 1 || _task_tray_cmd == 2 || _task_tray_cmd == 3 || _task_tray_cmd == 4 || _task_tray_cmd == 5 || _task_tray_cmd == 6 || _task_tray_cmd == 7 || _task_tray_cmd == 8)
        if(_task_tray_cmd == 1 || _task_tray_cmd == 2 || _task_tray_cmd == 3 || _task_tray_cmd == 4 || _task_tray_cmd == 5 || _task_tray_cmd == 6 || _task_tray_cmd == 7 || _task_tray_cmd == 8)
		{
			if(_pre_tray_cmd !=_task_tray_cmd)
			{
				_CurrentTask = Test_Task;
				_CurrentState = Front_Table_Lane;
				_init_time = time;			
				cout << "Message Received" <<endl;
				_PreviousState = Ready;

				_task_state = 0;
			}
			_pre_tray_cmd = _task_tray_cmd;
		}
        else if(_task_tray_cmd == 9 || _task_tray_cmd == 10 || _task_tray_cmd == 11 || _task_tray_cmd == 12 || _task_tray_cmd == 13 || _task_tray_cmd == 14 || _task_tray_cmd == 15 || _task_tray_cmd == 16)
		{
			if(_pre_tray_cmd !=_task_tray_cmd)
			{
				_CurrentTask = Test_Task;
				_CurrentState = Back_Table_Lane;
				_init_time = time;			
				cout << "Message Received" <<endl;
				_PreviousState = Ready;

				_task_state = 0;
			}
			_pre_tray_cmd = _task_tray_cmd;
		}
        else if(_task_tray_cmd == 0)
        {
            _pre_tray_cmd = _task_tray_cmd;
        }
//
        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(0.0, false);
        // read_state_deg(_q_goal_forward);

        Scara_backward.write_cmd_from_FSM(_operation_time, _q_goal_backward, _x_goal_backward, _bool_ee_control);
        Linear_backward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_backward.write_cmd_from_FSM(0.0,false);


        //_task_state = 2;
    }


/////////////////////////////////////////////////////////// IK State /////////////////////////////////////////////////////////////
   
    else if(_CurrentState == IK_State)
    {
        _operation_time = 5.0;
        
        /*
        std::cout << "Please enter x,y,z,a values : ";
		std::cin >> IK_x;
		std::cin >> IK_y;
		std::cin >> IK_z;
        std::cin >> IK_a;
		std::cout << "x = " << IK_x << ", " << "y = " << IK_y << ", " << "z = " << IK_z << ", " << "a = " << IK_a << std::endl; 
        */

		// Scara_forward.calc_iversekinematics( -0.10, 0.20, 180.0*DEG2RAD); //solve IK to get _q[1]~[3]
        Scara_forward.calc_iversekinematics( 0.17, -0.74, -90.0*DEG2RAD); //solve IK to get _q[1]~[3]
		_q_goal_forward[0] = Scara_forward._x_ee_goal_local[0];
        _q_goal_forward[1] = Scara_forward._x_ee_goal_local[1];
        _q_goal_forward[2] = Scara_forward._x_ee_goal_local[2];
        //_height_goal_linear = 1.45 - 1.217;
        _height_goal_linear = 1.45 - 1.187;
        
        
        // std::cout << "_q_goal_forward" << std::endl;
        // std::cout << _q_goal_forward[0]*RAD2DEG << std::endl;
        // std::cout << _q_goal_forward[1]*RAD2DEG << std::endl;
        // std::cout << _q_goal_forward[2]*RAD2DEG << std::endl;
        
        
        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        //Linear_forward.write_cmd_from_FSM(10, IK_z);
    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////  Forward Scara  //////////////////////////////////////////////////////////////

     // Front Table lane
    else if(_CurrentState == Front_Table_Lane)
    {
        if(_PreviousState == Ready)
            {
                if(_task_tray_cmd == 1)
                {
                    _operation_time = 8.0;
                }
                else if(_task_tray_cmd == 2)
                {
                    _operation_time = 6.0;
                }
                else if(_task_tray_cmd == 3)
                {
                    _operation_time = 6.0;
                }
                else if(_task_tray_cmd == 4)
                {
                    _operation_time = 9.0;
                }
                else if(_task_tray_cmd == 5)
                {
                    _operation_time = 10.0;
                }
                else if(_task_tray_cmd == 6)
                {
                    _operation_time = 8.0;
                }
                else if(_task_tray_cmd == 7)
                {
                    _operation_time = 5.0;
                }
                else if(_task_tray_cmd == 8)
                {
                    _operation_time = 10.0;
                }
            }

            else if(_PreviousState == Item_Move)
            {
                _operation_time = 1.0;
            }

        //_operation_time = 8.0;
        
		if(time < _init_time + _operation_time)
        {
            boxlane_initial_desired(_task_tray_cmd);
        }
    
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Ready)
            {
                _CurrentState = Front_Table;
                _init_time = time;          
                _PreviousState = Front_Table_Lane;
            }
            else if(_PreviousState == Item_Move)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;          
                _PreviousState = Front_Table_Lane;
            }
        }       

        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        // read_state_deg(_q_goal_forward);
        tray_forward.write_cmd_from_FSM(0.0,false);
        tray_backward.write_cmd_from_FSM(0.0,false);
    }


     // Front Table
    else if(_CurrentState == Front_Table)
    {
        _operation_time = 1.0; 

        if(time >= _init_time && time < _init_time + _operation_time)
        {
		    boxlane_table_desired(_task_tray_cmd);
        }       
        else if(time >= _init_time + _operation_time)// + 3.0)
        {
            if(_CurrentTask == Test_Task)
            {
                _CurrentState = Item_Move;
                _init_time = time;              
            }
            _PreviousState = Front_Table;
        }   
        Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        // read_state_deg(_q_goal_forward);
        tray_forward.write_cmd_from_FSM(0.0,false);
        tray_backward.write_cmd_from_FSM(0.0,false);
    }
   

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////  Backward Scara  //////////////////////////////////////////////////////////////



    // Back Table lane
    else if(_CurrentState == Back_Table_Lane)
    {
        _operation_time = 10.0;
        
		if(time < _init_time + _operation_time)
        {
            boxlane_initial_desired(_task_tray_cmd);
        }
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Ready)
            {
                _CurrentState = Back_Table;
                _init_time = time;          
                _PreviousState = Back_Table_Lane;
            }
            else if(_PreviousState == Item_Move)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;          
                _PreviousState = Back_Table_Lane;
            }
        }       

        Scara_backward.write_cmd_from_FSM(_operation_time, _q_goal_backward, _x_goal_backward, _bool_ee_control);
        Linear_backward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_backward.write_cmd_from_FSM(0.0,false);
    }

   

     // Back Table
    else if(_CurrentState == Back_Table)
    {
        _operation_time = 5.0; 

        if(time >= _init_time && time < _init_time + _operation_time)
        {
		    boxlane_initial_desired(_task_tray_cmd);
        }       
        else if(time >= _init_time + _operation_time + 3.0)
        {
            if(_CurrentTask == Test_Task)
            {
                _CurrentState = Item_Move;
                _init_time = time;              
            }
            _PreviousState = Back_Table;
        }   
        Scara_backward.write_cmd_from_FSM(_operation_time, _q_goal_backward, _x_goal_backward, _bool_ee_control);
        Linear_backward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_backward.write_cmd_from_FSM(0.0,false);
    }
   

 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Item Move

/*
else if(_CurrentState == Item_Move)
    {
        int tray_vel= 0.0;

        if( _PreviousState == Front_Table )
        {
            if( _task_tray_cmd == 1 || _task_tray_cmd == 2 || _task_tray_cmd == 3 || _task_tray_cmd == 4 || _task_tray_cmd == 5 )
            {
                tray_vel = 100;
            }
            else if( _task_tray_cmd == 6 || _task_tray_cmd == 7 || _task_tray_cmd == 8 )
            {    
                tray_vel = -100;
            }

            _CurrentState = Front_Table_Lane;
            _init_time = time; 
        }
        else if( _PreviousState == Back_Table )
        {
            if( _task_tray_cmd == 9 || _task_tray_cmd == 10 || _task_tray_cmd == 11 || _task_tray_cmd == 12 || _task_tray_cmd == 13 )
            {
                tray_vel = 100;
            }
            else if( _task_tray_cmd == 14 || _task_tray_cmd == 15 || _task_tray_cmd == 16 )
            {    
                tray_vel = -100;
            }

            _CurrentState = Back_Table_Lane;
            _init_time = time; 
        }           
        else if(_PreviousState == Drop_Point)
        {
            _CurrentState = Drop_Lane;
            _init_time = time;
        }           

        _PreviousState = Item_Move;
        

        //Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        //Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(tray_vel, true);
        tray_backward.write_cmd_from_FSM(0.0,false);
    }
*/


    else if(_CurrentState == Item_Move)
    {
        int tray_vel= 0.0;
        _operation_time = 8.0; 

        if(time < _init_time + _operation_time)
        {
            if( _PreviousState == Front_Table )
            {
                if( _task_tray_cmd == 1 || _task_tray_cmd == 2 || _task_tray_cmd == 3 || _task_tray_cmd == 4 || _task_tray_cmd == 5 )
                {
                    tray_vel = 100;
                }
                else if( _task_tray_cmd == 6 || _task_tray_cmd == 7 || _task_tray_cmd == 8 )
                {    
                    tray_vel = -100;
                }
            }
            else if( _PreviousState == Back_Table )
            {
                if( _task_tray_cmd == 9 || _task_tray_cmd == 10 || _task_tray_cmd == 11 || _task_tray_cmd == 12 || _task_tray_cmd == 13 )
                {
                    tray_vel = 100;
                }
                else if( _task_tray_cmd == 14 || _task_tray_cmd == 15 || _task_tray_cmd == 16 )
                {    
                    tray_vel = -100;
                }
            } 
            else if( _PreviousState == Drop_Point )  
            {
                tray_vel = -100;
            }        
        }
        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Front_Table)
            {
                _CurrentState = Front_Table_Lane;
                _init_time = time;              
            }
            else if(_PreviousState == Back_Table)
            {
                _CurrentState = Back_Table_Lane;
                _init_time = time;              
            }
            else if(_PreviousState == Drop_Point)
            {
                _CurrentState = Drop_Lane;
                _init_time = time;
            }           
            _PreviousState = Item_Move;
        }       

        //Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        //Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(tray_vel, true);
        tray_backward.write_cmd_from_FSM(0.0,false);
    }



    // Drop Lane
    else if(_CurrentState == Drop_Lane)
    {
        if(_PreviousState == Front_Table_Lane)
            {
                if(_task_tray_cmd == 1)
                {
                    _operation_time = 6.0;
                }
                else if(_task_tray_cmd == 2)
                {
                    _operation_time = 5.5;
                }
                else if(_task_tray_cmd == 3)
                {
                    _operation_time = 5.5;
                }
                else if(_task_tray_cmd == 4)
                {
                    _operation_time = 5.5;
                }
                else if(_task_tray_cmd == 5)
                {
                    _operation_time = 6.0;
                }
                else if(_task_tray_cmd == 6)
                {
                    _operation_time = 8.0;
                }
                else if(_task_tray_cmd == 7)
                {
                    _operation_time = 5.0;
                }
                else if(_task_tray_cmd == 8)
                {
                    _operation_time = 6.0;
                }
            }

            else if(_PreviousState == Item_Move)
            {
                _operation_time = 2.0;
            }
            

        
        //_operation_time = 10.0;
        
		if(time < _init_time + _operation_time)
        {
            if(_PreviousState == Front_Table_Lane)            
            {
                //_q_goal_forward[0] = -73.0*DEG2RAD; //q0
		        //_q_goal_forward[1] = -20.0*DEG2RAD; //q1
		        //_q_goal_forward[2] = -87.0*DEG2RAD; //q2

                //_q_goal_forward[0] = -18.0*DEG2RAD; //q0 For Test
		        //_q_goal_forward[1] = -90.0*DEG2RAD; //q1
		        //_q_goal_forward[2] = -25.0*DEG2RAD; //q2
                


                // if(_q_goal_forward[0] == -9.12*DEG2RAD && _q_goal_forward[1] == 147.43*DEG2RAD && _q_goal_forward[2] = 41.97*DEG2RAD)
                // {
                //     _q_goal_forward[0] = 30.0*DEG2RAD; //q0
		        //     _q_goal_forward[1] = -135.0*DEG2RAD; //q1
		        //     _q_goal_forward[2] = 15.0*DEG2RAD; //q2
		        //     _height_goal_linear = 1.45 - 1.187;
                // }

                _q_goal_forward[0] = 30.0*DEG2RAD; //q0
		        _q_goal_forward[1] = -135.0*DEG2RAD; //q1
		        _q_goal_forward[2] = 15.0*DEG2RAD; //q2
		        _height_goal_linear = 1.45 - 1.187;

                Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
                Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
                // read_state_deg(_q_goal_forward);
            }
            else if(_PreviousState == Back_Table_Lane)
            {
                _q_goal_backward[0] = -73.0*DEG2RAD; //q0
		        _q_goal_backward[1] = -20.0*DEG2RAD; //q1
		        _q_goal_backward[2] = -87.0*DEG2RAD; //q2
		        _height_goal_linear = 1.45 - 1.237;

                Scara_backward.write_cmd_from_FSM(_operation_time, _q_goal_backward, _x_goal_backward, _bool_ee_control);
                Linear_backward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
                // read_state_deg(_q_goal_forward);
            }
            else if(_PreviousState == Item_Move)
            {
                //_q_goal_forward[0] = -18.0*DEG2RAD; //q0 for test
		        //_q_goal_forward[1] = -90.0*DEG2RAD; //q1
		        //_q_goal_forward[2] = -25.0*DEG2RAD; //q2
                
                _q_goal_forward[0] = 30.0*DEG2RAD; //q0
		        _q_goal_forward[1] = -135.0*DEG2RAD; //q1
		        _q_goal_forward[2] = 15.0*DEG2RAD; //q2
		        _height_goal_linear = 1.45 - 1.187;

                Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
                Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
                // read_state_deg(_q_goal_forward);
            }
            
        }

        else if(time >= _init_time + _operation_time)
        {
            if(_PreviousState == Item_Move)
            {
                _CurrentState = Ready;
                _task_tray_cmd = 0;
                _init_time = time;          
                _PreviousState = Drop_Lane;

                _task_state = 2;
            }
            else if(_PreviousState == Front_Table_Lane)
            {
                _CurrentState = Drop_Point;
                _init_time = time;          
                _PreviousState = Front_Table_Lane;

                Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
                Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
                // read_state_deg(_q_goal_forward);
            }
            else if(_PreviousState == Back_Table_Lane)
            {
                _CurrentState = Drop_Point;
                _init_time = time;          
                _PreviousState = Back_Table_Lane;

                Scara_backward.write_cmd_from_FSM(_operation_time, _q_goal_backward, _x_goal_backward, _bool_ee_control);
                Linear_backward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
                // read_state_deg(_q_goal_forward);
            }
        }       

        //Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
        //Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
        tray_forward.write_cmd_from_FSM(0.0,false);
        tray_backward.write_cmd_from_FSM(0.0,false);
        // read_state_deg(_q_goal_forward);
    }

    // Drop Point
    else if(_CurrentState == Drop_Point)
    {
        _operation_time = 2.0;
        if(time < _init_time + _operation_time)
        {
            if(_PreviousState == Front_Table_Lane)
            {
                Scara_forward.calc_iversekinematics( _target_position_from_vision[0], _target_position_from_vision[1], -90.0*DEG2RAD);
                //Scara_forward.calc_iversekinematics( -0.08, -0.63, -135.0*DEG2RAD); //solve IK to get _q[1]~[3]
                //Scara_forward.calc_iversekinematics( 0.20, -0.63, -90.0*DEG2RAD); //solve IK to get _q[1]~[3]
            
                
                //    _q_goal_forward[0] = Scara_forward._x_ee_goal_local[0];
                //    _q_goal_forward[1] = Scara_forward._x_ee_goal_local[1];
                //    _q_goal_forward[2] = Scara_forward._x_ee_goal_local[2];


                 if(_target_position_from_vision[0] < 0.0)
                 {
                     _q_goal_forward[0] = 30.0*DEG2RAD; //q0
		             _q_goal_forward[1] = -135.0*DEG2RAD; //q1
		             _q_goal_forward[2] = 15.0*DEG2RAD; //q2
                 }
                 else if(_target_position_from_vision[0] >= 0.0)
                 {
                     _q_goal_forward[0] = Scara_forward._x_ee_goal_local[0];
                     _q_goal_forward[1] = Scara_forward._x_ee_goal_local[1];
                     _q_goal_forward[2] = Scara_forward._x_ee_goal_local[2];
                 }
                _height_goal_linear = 1.45 - 1.187;
                //_height_goal_linear = 1.45 - 1.217;

                Scara_forward.write_cmd_from_FSM(_operation_time, _q_goal_forward, _x_goal_forward, _bool_ee_control);
                Linear_forward.write_cmd_from_FSM(_operation_time, _height_goal_linear); 
                // read_state_deg(_q_goal_forward);
            }
            else if(_PreviousState == Back_Table_Lane)
            {
                _q_goal_backward[0] = -73.0*DEG2RAD; //q0
		        _q_goal_backward[1] = -20.0*DEG2RAD; //q1
		        _q_goal_backward[2] = 93.0*DEG2RAD; //q2
		        _height_goal_linear = 1.45 - 1.207;

                Scara_backward.write_cmd_from_FSM(_operation_time, _q_goal_backward, _x_goal_backward, _bool_ee_control);
                Linear_backward.write_cmd_from_FSM(_operation_time, _height_goal_linear);
            } 
        }
        else if(time >= _init_time + _operation_time)
        {
            _CurrentState = Item_Move;
            _init_time = time;          
            _PreviousState = Drop_Point;
        }       

        tray_forward.write_cmd_from_FSM(0.0,false);
        tray_backward.write_cmd_from_FSM(0.0,false);
    }
}

// void CUController::read_state_deg(double jointangle[]) //read from ROS
// {
//     
//     _ee_pos[0] = 0.35 * cos(jointangle[0]) + 0.35 * cos(jointangle[0] + jointangle[1]) + 0.2 * cos(jointangle[0] + jointangle[1] + jointangle[2]); // m
// 	_ee_pos[1] = 0.35 * sin(jointangle[0]) + 0.35 * sin(jointangle[0] + jointangle[1]) + 0.2 * sin(jointangle[0] + jointangle[1] + jointangle[2]); // m
// 	_ee_pos[2] = jointangle[0] + jointangle[1] + jointangle[2]; // rad
// }


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CScara::CScara()
{
    //initialize
    for(int i=0; i<3; i++)
    {
        _q[i] = 0.0;
        _q_goal[i] = 0.0;
        _x_ee_local[i] = 0.0;
        _x_ee_goal_local[i] = 0.0;
        
    }
    _bool_ee_control = false;
    _time_motion = 5.0;
    _link_length[0] = 0.35;
    _link_length[1] = 0.35;
    _link_length[2] = 0.2;

    _rev = true;

    x = 0.0;
    y = 0.0;
    z = 0.0;
    cos_q2 = 0.0;
    sin_q2_1 = 0.0;
    sin_q2_2 = 0.0;
    q1_1 = 0.0;
    q1_2 = 0.0;
    q2_1 = 0.0;
    q2_2 = 0.0;
    q3_1 = 0.0;
    q3_2 = 0.0;
    k1_1 = 0.0;
    k1_2 = 0.0;
    k2_1 = 0.0;
    k2_2 = 0.0;
    gamma_1 = 0.0;
    gamma_2 = 0.0;
}

CScara::~CScara()
{
}

void CScara::update_forward_kinematics()
{
    _x_ee_local[0] = _link_length[0] * cos(_q[0]) + _link_length[1] * cos(_q[0] + _q[1]) + _link_length[2] * cos(_q[0] + _q[1] + _q[2]); // m
	_x_ee_local[1] = _link_length[0] * sin(_q[0]) + _link_length[1] * sin(_q[0] + _q[1]) + _link_length[2] * sin(_q[0] + _q[1] + _q[2]); // m
	_x_ee_local[2] = _q[0] + _q[1] + _q[2]; // rad

   
}

void CScara::calc_iversekinematics(double IK_x, double IK_y, double IK_a) 
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    cos_q2 = 0.0;
    sin_q2_1 = 0.0;
    sin_q2_2 = 0.0;
    q1_1 = 0.0;
    q1_2 = 0.0;
    q2_1 = 0.0;
    q2_2 = 0.0;
    q3_1 = 0.0;
    q3_2 = 0.0;
    k1_1 = 0.0;
    k1_2 = 0.0;
    k2_1 = 0.0;
    k2_2 = 0.0;
    gamma_1 = 0.0;
    gamma_2 = 0.0;

	x = IK_x - _link_length[2] * cos(IK_a);
	y = IK_y - _link_length[2] * sin(IK_a);
    
	cos_q2 = (pow(x, 2) + pow(y, 2) - pow(_link_length[0], 2) - pow(_link_length[1], 2)) / (2.0 * _link_length[0] * _link_length[1]);

	if (abs(cos_q2) > 1) {
		std::cout << "Out of Workspace." << endl;
		//_x_ee_goal_local[0] = 0.0;// {0.0, 0.0, 0.0 };
		//_x_ee_goal_local[1] = 0.0;
		//_x_ee_goal_local[2] = 0.0;
        _x_ee_goal_local[0] = -18.0*DEG2RAD; //q0
		_x_ee_goal_local[1] = -90.0*DEG2RAD; //q1
		_x_ee_goal_local[2] = -25.0*DEG2RAD; //q2
        error_code_ik = 2502;
	}
	else {

		sin_q2_1 = sqrt(1 - pow(cos_q2, 2));
		sin_q2_2 = -sqrt(1 - pow(cos_q2, 2));

		q2_1 = atan2(sin_q2_1, cos_q2);
		q2_2 = atan2(sin_q2_2, cos_q2);
		
		k1_1 = _link_length[1] * cos(q2_1) + _link_length[0];
		k1_2 = _link_length[1] * cos(q2_2) + _link_length[0];

		k2_1 = _link_length[1] * sin(q2_1);
		k2_2 = _link_length[1] * sin(q2_2);
		
		gamma_1 = atan2(k2_1, k1_1);
		gamma_2 = atan2(k2_2, k1_2);
		
		z = atan2(y, x);
		q1_1 = z - gamma_1;
		q1_2 = z - gamma_2;
		q3_1 = IK_a - q1_1 - q2_1;
		q3_2 = IK_a - q1_2 - q2_2;

        if (_rev == true)// 1 이냐 2냐 고를수 있음
		{
			_x_ee_goal_local[0] = range(q1_2);
            _x_ee_goal_local[1] = range(q2_2);
            _x_ee_goal_local[2] = range(q3_2);
		}
		else
		{
			_x_ee_goal_local[0] = range(q1_1);
            _x_ee_goal_local[1] = range(q2_1);
            _x_ee_goal_local[2] = range(q3_1);
		}
	}

}

double CScara::range(double angle) 
{
	while (angle > PI || angle <= -PI) 
	{
		if (angle > PI) 
		{
			angle = angle - 2 * PI;
		}
		else 
		{
			angle = angle + 2 * PI;
		}
	}
	return angle;
}


void CScara::read_state(double jointangle[]) //read from ROS
{
    _q[0] = jointangle[0];
    _q[1] = jointangle[1];
    _q[2] = jointangle[2];

    update_forward_kinematics();

   
}




void CScara::write_cmd_from_FSM(double motion_time, double jointangle_cmd[], double endeffector_cmd[], bool bool_endeffector_ctrl) //read from FSM
{
    _time_motion = motion_time;

    _q_goal[0] = jointangle_cmd[0]*RAD2DEG;
    _q_goal[1] = jointangle_cmd[1]*RAD2DEG;
    _q_goal[2] = jointangle_cmd[2]*RAD2DEG;

    _x_ee_goal_local[0] = endeffector_cmd[0];
    _x_ee_goal_local[1] = endeffector_cmd[1];
    _x_ee_goal_local[2] = endeffector_cmd[2];

    _bool_ee_control = bool_endeffector_ctrl;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CLinear::CLinear()
{
    _height = 1.45;
    _height_goal = 1.45;
    _time_motion = 10.0;
}

CLinear::~CLinear()
{
    
}

void CLinear::read_state(double height) //read robot state
{
    _height = 1.45 - height; //m, initial height is 1.45 = 0 in encoder
}
    
void CLinear::write_cmd_from_FSM(double motion_time, double height_cmd) //read command
{
    _height_goal = height_cmd - 1.45; //height based on the motor coordinate (0~-1.45)
    _time_motion = motion_time;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CTray::CTray()
{
    _velocity_cmd = 0.0;
    _bool_tray_cmd = false;
}

CTray::~CTray()
{
}

void CTray::write_cmd_from_FSM(double velocity_cmd, bool bool_tray_motion_cmd)
{
    _velocity_cmd = velocity_cmd;
    _bool_tray_cmd = bool_tray_motion_cmd;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CUController::boxlane_initial_desired(int tray_num)
{
    
    if(tray_num == 1)
    {
        _q_goal_forward[0] = -9.12*DEG2RAD; //q0
		_q_goal_forward[1] = 147.43*DEG2RAD; //q1
		_q_goal_forward[2] = 41.97*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.453; //z
    }
    else if(tray_num == 2)
    {
        _q_goal_forward[0] = -9.12*DEG2RAD; //q0
		_q_goal_forward[1] = 147.43*DEG2RAD; //q1
		_q_goal_forward[2] = 41.97*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.702; //z 
    }
    else if(tray_num == 3)
    {
        _q_goal_forward[0] = -9.12*DEG2RAD; //q0
		_q_goal_forward[1] = 147.43*DEG2RAD; //q1
		_q_goal_forward[2] = 41.97*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.944; //z  
    }
    else if(tray_num == 4)
    {
        _q_goal_forward[0] = -9.12*DEG2RAD; //q0
		_q_goal_forward[1] = 147.43*DEG2RAD; //q1
		_q_goal_forward[2] = 41.97*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.186; //z
    }
    else if(tray_num == 5)
    {
        _q_goal_forward[0] = -9.12*DEG2RAD; //q0
		_q_goal_forward[1] = 147.43*DEG2RAD; //q1
		_q_goal_forward[2] = 41.97*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.438; //z
    }
    else if(tray_num == 6)
    {
        _q_goal_forward[0] = 27.2*DEG2RAD; //q0
		_q_goal_forward[1] = -140.5*DEG2RAD; //q1
	    _q_goal_forward[2] = 116.03*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.618; //z 
    }
    else if(tray_num == 7)
    {
        //Scara_forward.calc_iversekinematics( -0.08, -0.17, -180.0*DEG2RAD);
        //_q_goal_forward[0] = Scara_forward._x_ee_goal_local[0];
        //_q_goal_forward[1] = Scara_forward._x_ee_goal_local[1];
        //_q_goal_forward[2] = Scara_forward._x_ee_goal_local[2];
        _q_goal_forward[0] = 27.2*DEG2RAD; //q0
		_q_goal_forward[1] = -140.5*DEG2RAD; //q1
	    _q_goal_forward[2] = 116.03*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.023; //z 
    }
    else if(tray_num == 8)
    {
        _q_goal_forward[0] = 27.2*DEG2RAD; //q0
		_q_goal_forward[1] = -140.5*DEG2RAD; //q1
	    _q_goal_forward[2] = 116.03*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.43; //z 
    }


    else if(tray_num == 9)
    {
        _q_goal_backward[0] = -11.84*DEG2RAD; //q0
		_q_goal_backward[1] = 146.23*DEG2RAD; //q1
		_q_goal_backward[2] = 45.67*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.637; //z 
    }
    else if(tray_num == 10)
    {
        _q_goal_backward[0] = -11.84*DEG2RAD; //q0
		_q_goal_backward[1] = 146.23*DEG2RAD; //q1
		_q_goal_backward[2] = 45.67*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.037; //z 
    }
    else if(tray_num == 11)
    {
        _q_goal_backward[0] = -11.84*DEG2RAD; //q0
		_q_goal_backward[1] = 146.23*DEG2RAD; //q1
		_q_goal_backward[2] = 45.67*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.43; //z 
    }
    else if(tray_num == 12)
    {
        _q_goal_backward[0] = 10.91*DEG2RAD; //q0
		_q_goal_backward[1] = -147.88*DEG2RAD; //q1
		_q_goal_backward[2] = 134.98*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.474; //z
    }
    else if(tray_num == 13)
    {
        _q_goal_backward[0] = 10.91*DEG2RAD; //q0
		_q_goal_backward[1] = -147.88*DEG2RAD; //q1
		_q_goal_backward[2] = 134.98*DEG2RAD; //q2
	    _height_goal_linear = 1.45 - 0.716; //z   
    }
    else if(tray_num == 14)
    {
        _q_goal_backward[0] = 8.91*DEG2RAD; //q0
		_q_goal_backward[1] = -147.88*DEG2RAD; //q1
		_q_goal_backward[2] = 137.98*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.023; //z 
    }
    else if(tray_num == 15)
    {
        _q_goal_backward[0] = 8.91*DEG2RAD; //q0
		_q_goal_backward[1] = -147.88*DEG2RAD; //q1
		_q_goal_backward[2] = 137.98*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.202; //z 
    }
    else if(tray_num == 16)
    {
        _q_goal_backward[0] = 8.91*DEG2RAD; //q0
		_q_goal_backward[1] = -147.88*DEG2RAD; //q1
		_q_goal_backward[2] = 137.98*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.43; //z 
    }
}

void CUController::boxlane_table_desired(int tray_num)
{
    /*
    if(tray_num == 1)
    {
        _q_goal_forward[0] = -9.12*DEG2RAD; //q0
		_q_goal_forward[1] = 147.43*DEG2RAD; //q1
		_q_goal_forward[2] = 41.97*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.453; //z
    }
    else if(tray_num == 2)
    {
        _q_goal_forward[0] = -9.12*DEG2RAD; //q0
		_q_goal_forward[1] = 147.43*DEG2RAD; //q1
		_q_goal_forward[2] = 41.97*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.702; //z 
    }
    else if(tray_num == 3)
    {
        _q_goal_forward[0] = -9.12*DEG2RAD; //q0
		_q_goal_forward[1] = 147.43*DEG2RAD; //q1
		_q_goal_forward[2] = 41.97*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.944; //z  
    }
    else if(tray_num == 4)
    {
        _q_goal_forward[0] = -9.12*DEG2RAD; //q0
		_q_goal_forward[1] = 147.43*DEG2RAD; //q1
		_q_goal_forward[2] = 41.97*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.186; //z
    }
    */
    if(tray_num == 1)
    {
        _q_goal_forward[0] = -3.21*DEG2RAD; //q0
		_q_goal_forward[1] = 148.45*DEG2RAD; //q1
		_q_goal_forward[2] = 38.07*DEG2RAD; //q2

        // _q_goal_forward[0] = -9.12*DEG2RAD; //q0
		// _q_goal_forward[1] = 147.43*DEG2RAD; //q1
		// _q_goal_forward[2] = 41.97*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.453; //z
        _check_table[0] = true;
    }
    else if(tray_num == 2)
    {
        _q_goal_forward[0] = -3.21*DEG2RAD; //q0
		_q_goal_forward[1] = 148.45*DEG2RAD; //q1
		_q_goal_forward[2] = 38.07*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.702; //z 
        _check_table[1] = true;
    }
    else if(tray_num == 3)
    {
        _q_goal_forward[0] = -3.21*DEG2RAD; //q0
		_q_goal_forward[1] = 148.45*DEG2RAD; //q1
		_q_goal_forward[2] = 38.07*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.944; //z 0.948
        _check_table[2] = true;
        
    }
    else if(tray_num == 4)
    {
        _q_goal_forward[0] = -3.21*DEG2RAD; //q0
		_q_goal_forward[1] = 148.45*DEG2RAD; //q1
		_q_goal_forward[2] = 38.07*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.186; //z
        _check_table[3] = true;
    }
    
    else if(tray_num == 5)
    {
        _q_goal_forward[0] = -9.12*DEG2RAD; //q0
		_q_goal_forward[1] = 147.43*DEG2RAD; //q1
		_q_goal_forward[2] = 43.97*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.30; //z
        _check_table[4] = true;
    }
    else if(tray_num == 6)
    {
        _q_goal_forward[0] = 27.2*DEG2RAD; //q0
		_q_goal_forward[1] = -140.5*DEG2RAD; //q1
	    _q_goal_forward[2] = 116.03*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.618; //z 
        _check_table[5] = true;
    }
    else if(tray_num == 7)
    {
        //_q_goal_forward[0] = 6.27*DEG2RAD; //q0
		//_q_goal_forward[1] = -148.79*DEG2RAD; //q1
	    //_q_goal_forward[2] = 141.43*DEG2RAD; //q2

        //Scara_forward.calc_iversekinematics( -0.08, -0.17, -180.0*DEG2RAD);
        //_q_goal_forward[0] = Scara_forward._x_ee_goal_local[0];
        //_q_goal_forward[1] = Scara_forward._x_ee_goal_local[1];
        //_q_goal_forward[2] = Scara_forward._x_ee_goal_local[2];
        _q_goal_forward[0] = 27.2*DEG2RAD; //q0
		_q_goal_forward[1] = -140.5*DEG2RAD; //q1
	    _q_goal_forward[2] = 116.03*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.023; //z 
        _check_table[6] = true;
    }
    else if(tray_num == 8)
    {
        _q_goal_forward[0] = 27.2*DEG2RAD; //q0
		_q_goal_forward[1] = -140.5*DEG2RAD; //q1
	    _q_goal_forward[2] = 116.03*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.43; //z 
        _check_table[7] = true;
    }


    else if(tray_num == 9)
    {
        _q_goal_backward[0] = -11.84*DEG2RAD; //q0
		_q_goal_backward[1] = 146.23*DEG2RAD; //q1
		_q_goal_backward[2] = 45.67*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.637; //z 
        _check_table[8] = true;
    }
    else if(tray_num == 10)
    {
        _q_goal_backward[0] = -11.84*DEG2RAD; //q0
		_q_goal_backward[1] = 146.23*DEG2RAD; //q1
		_q_goal_backward[2] = 45.67*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.037; //z 
        _check_table[9] = true;
    }
    else if(tray_num == 11)
    {
        _q_goal_backward[0] = -11.84*DEG2RAD; //q0
		_q_goal_backward[1] = 146.23*DEG2RAD; //q1
		_q_goal_backward[2] = 45.67*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.43; //z 
        _check_table[10] = true;
    }
    else if(tray_num == 12)
    {
        _q_goal_backward[0] = 10.91*DEG2RAD; //q0
		_q_goal_backward[1] = -147.88*DEG2RAD; //q1
		_q_goal_backward[2] = 134.98*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 0.474; //z
        _check_table[11] = true;
    }
    else if(tray_num == 13)
    {
        _q_goal_backward[0] = 10.91*DEG2RAD; //q0
		_q_goal_backward[1] = -147.88*DEG2RAD; //q1
		_q_goal_backward[2] = 134.98*DEG2RAD; //q2
	    _height_goal_linear = 1.45 - 0.716; //z  
        _check_table[12] = true; 
    }
    else if(tray_num == 14)
    {
        _q_goal_backward[0] = 8.91*DEG2RAD; //q0
		_q_goal_backward[1] = -147.88*DEG2RAD; //q1
		_q_goal_backward[2] = 137.98*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.023; //z 
        _check_table[13] = true;
    }
    else if(tray_num == 15)
    {
        _q_goal_backward[0] = 8.91*DEG2RAD; //q0
		_q_goal_backward[1] = -147.88*DEG2RAD; //q1
		_q_goal_backward[2] = 137.98*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.202; //z 
        _check_table[14] = true;
    }
    else if(tray_num == 16)
    {
        _q_goal_backward[0] = 8.91*DEG2RAD; //q0
		_q_goal_backward[1] = -147.88*DEG2RAD; //q1
		_q_goal_backward[2] = 137.98*DEG2RAD; //q2
		_height_goal_linear = 1.45 - 1.43; //z 
        _check_table[15] = true;
    }
}
