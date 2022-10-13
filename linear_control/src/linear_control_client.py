# -*- coding:utf-8 -*-
# TODO: Add initial position cnt
import rospy
import math
from linear_function import Linear as linear_func
from linear_control.msg import linear_cmd, linear_status
from std_msgs.msg import String, Bool
import threading
import time
# Linear (z-axis)
class linear_Client():
    def __init__(self):
        rospy.init_node('linear_Client')
        self.linear_status_pub = rospy.Publisher('/linear_status', linear_status, queue_size=100)
        self.linear_cmd = None
        self.linear_status = linear_status()
        self.linear_func = linear_func()
        rospy.loginfo(":: Linear Client Started ::")
        self.thread_pause = False
        self.init_pulse = [0.0, 0.0]
        self.linear_func.index_setting([1, 2])
        # ===for motion===     
        self.position1_list = []
        self.position2_list = []
        self.time_list = []
        # ================

    def msg_callback(self, data):
        self.linear_cmd = data
    
    def get_linear_msg(self):
        data = rospy.wait_for_message('/linear_control', linear_cmd)
        return data

    def pub_linear_status(self):
        while True:
            t1 = time.time()
            node_id = [1,2]
            error_flag = False
            status = []
            position = []
            raw_pulse = []
            pulse = []

            for i in node_id:
                try:
                    status.append(self.linear_func.read_status(i))
                    raw_pulse, _ = self.linear_func.read_position(i)
                except:
                    print("Position read error")
                    error_flag = True
                    break

                one_meter = 68268966

                tune_pulse = raw_pulse - self.init_pulse[i-1]
                m_val = round(tune_pulse/one_meter,3)
                position.append(m_val)
                pulse.append(raw_pulse)

            if None in status:
                error_flag = True
            if error_flag:
                continue
            else:
                self.pub_status = self.linear_status
                self.pub_status.servo = [status[0]['servo'],status[1]['servo']]
                self.pub_status.alarm = [status[0]['alarm'],status[1]['alarm']]
                self.pub_status.warn = [status[0]['warn'],status[1]['warn']]
                self.pub_status.ready = [status[0]['ready'],status[1]['ready']]
                self.pub_status.inpos1 = [status[0]['inpos1'],status[1]['inpos1']]
                self.pub_status.zspd = [status[0]['zspd'],status[1]['zspd']]
                self.pub_status.current_position = [position[0], position[1]]
                self.pub_status.current_pulse = [pulse[0], pulse[1]]
                self.linear_status_pub.publish(self.pub_status)
                t2 = time.time()
                #print("status", t2-t1)
                break

    def check_ready(self, node):
        check_ready_list = []
        result = True
        for i in node:
            if (self.pub_status.servo[i-1]==0 and  self.pub_status.alarm[i-1]==0 and self.pub_status.warn[i-1]==0
                and self.pub_status.ready[i-1]==1 and self.pub_status.inpos1[i-1]==1 and self.pub_status.zspd[i-1]==1):
                check_ready_list.append(True)
            else:
                check_ready_list.append(False)

        for i in check_ready_list:
            result = result and i
        return result

    def control(self):
        while True:
            recv_cmd = self.get_linear_msg()
            if recv_cmd == None:
                print("[LINEAR::WARN] Command is None")
            else:
                if recv_cmd.cmd == "on":
                    node_id = recv_cmd.node
                    for i in node_id:
                        self.linear_func.sv_on(i)
                        self.linear_func.flush()
                    print("[LINEAR::SV_ON]")

                elif recv_cmd.cmd == "off":
                    node_id = recv_cmd.node
                    for i in node_id:
                        self.linear_func.sv_off(i)
                        self.linear_func.flush()
                    print("[LINEAR::SV_OFF]")
                
                elif recv_cmd.cmd == "stop":
                    node_id = recv_cmd.node
                    for i in node_id:
                        self.linear_func.stop(i)
                        self.linear_func.flush()
                    print("[LINEAR::STOP]")

                elif recv_cmd.cmd == "emg_on":
                    self.linear_func.emg_on(0)
                    self.linear_func.flush()
                    print("[LINEAR::EMG_ON]")
                    
                elif recv_cmd.cmd == "emg_off":
                    self.linear_func.emg_off(0)
                    self.linear_func.flush()
                    print("[LINEAR::EMG_OFF]")

                elif recv_cmd.cmd == "reset_alarm":
                    self.linear_func.alarm_reset(0)
                    self.linear_func.flush()
                    print("[LINEAR::RESET_ALARM]")
 
                elif recv_cmd.cmd == "status":
                    self.pub_linear_status()

                elif recv_cmd.cmd == "joint":
                    t_1 = time.time()
                    self.pub_linear_status()
                    target_position = [recv_cmd.position1, recv_cmd.position2]
                    limit_flag1 = True
                    limit_flag2 = True                    
                    
                    if recv_cmd.position1 > 1.45:
                        limit_flag1 = False
                    elif recv_cmd.position1 < 0:
                        limit_flag1 = False

                    if recv_cmd.position2 > 1.45:
                        limit_flag2 = False
                    elif recv_cmd.position2 < 0:
                        limit_flag2 = False
             
                    limit_flag = limit_flag1 and limit_flag2

                    if limit_flag:
                        target_time = recv_cmd.time
                        node_id = recv_cmd.node
                        control_flag = self.check_ready(node_id)
                        t_2 = time.time()
                        if control_flag:
                            for i in node_id:
                                init_pulse = self.init_pulse[i-1]
                                raw_pulse = self.pub_status.current_pulse[i-1]
                                current_position = self.pub_status.current_position[i-1]
                                self.linear_func.joint_control(raw_pulse, init_pulse, current_position, i, target_position[i-1], target_time)
                            self.linear_func.start(0)
                            self.linear_func.flush()
                            t_3 = time.time()
                            print("control_time:",t_3-t_1)
                            t_start = time.time()
                            while True:
                                self.pub_linear_status()
                                inpos_result = True
                                for i in node_id:
                                    inpos = self.pub_status.inpos1[i-1]
                                    inpos_result = inpos_result and inpos
                                if inpos_result:
                                    t_finish = time.time()
                                    print("operating_time:", t_finish-t_start)
                                    break
                            print("[LINEAR::JOINT]")
                        else:
                            print("[LINEAR::WARN] Driver is not ready.")
                    else:
                        print("[LINEAR::WARN] Joint limit.")

                        del self.position1_list [:]
                        del self.position2_list [:]
                        del self.time_list [:]

if __name__=='__main__':
    try:
        LC = linear_Client()
        LC.control()


    except rospy.ROSInterruptException:
        pass
