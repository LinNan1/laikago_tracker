#!/usr/bin/python
import rospy
import message_filters
import sys
from std_msgs.msg import Float32,String
from geometry_msgs.msg import Point
from laikago_msgs.msg import HighCmd, HighState
from simple_pid import PID
from laikago_command_handle import laikago_command_handle
from enum import Enum
from math import sin, cos, pi
class TrackFsm:
    def __init__(self):
        self.target_uv_sub = message_filters.Subscriber('/laikago_tracker/target_uv',Point)
        self.distance_sub = message_filters.Subscriber('/laikago_tracker/target_distance',Float32)
        self.state_sub = message_filters.Subscriber('/laikago_real/high_state',HighState)
        self.time_syn = message_filters.ApproximateTimeSynchronizer([self.target_uv_sub,self.distance_sub,self.state_sub], 10, 0.1, allow_headerless=True)
        self.time_syn.registerCallback(self.FSM_callback)

        # state
        self.FSM_state =  Enum('state', ('INIT', 'WAIT_TARGET', 'STAND_STILL_TRACK', 'STAND_ROTATE_TRACK', 'REPLAN', 'SEEK_TARGET'))
        self.state = self.FSM_state.INIT

        # contol
        self.cmd_handle = laikago_command_handle()
        self.RecvHighROS = HighState()
        self.P = 0.0005
        self.I = 0
        self.D = 0.00002
        
        self.pid_yaw = PID(self.P, self.I, self.D, setpoint=320)
        self.pid_yaw.output_limits = (-0.1, 0.1)
        self.pid_pitch = PID(self.P, self.I, self.D, setpoint=240)
        self.pid_pitch.output_limits = (-0.1, 0.1)

        self.pid_sideSpeed = PID(0.000002, 0.0, 0.0000002, setpoint=320)
        self.pid_sideSpeed.output_limits = (-1.0, 1.0)
        self.pid_rotateSpeed = PID(0.000103833984375, 0.0, 0.0, setpoint=320)
        self.pid_rotateSpeed.output_limits = (-1.0, 1.0)

        self.safe_distance = rospy.get_param('safe_distance') # m
        self.replan_distance = rospy.get_param('replan_distance') # m
        self.pid_forwardSpeed = PID(1.0, 0.0, 0.0, setpoint=-self.safe_distance)
        self.pid_forwardSpeed.output_limits = (-1.0,1.0)

        self.FSM_state_enter_first_time = rospy.Time.now()

        self.seek_curve_x = 0

        self.cmd_yaw = rospy.Publisher('/laikago_tracker/cmd_yaw',Float32,queue_size=10)
        self.cmd_pitch = rospy.Publisher('/laikago_tracker/cmd_pitch',Float32,queue_size=10)
        self.cmd_rotateSpeed = rospy.Publisher('/laikago_tracker/cmd_rotateSpeed',Float32,queue_size=10)
        self.FSM_state_pub = rospy.Publisher('/laikago_tracker/fsm_state',String,queue_size=10)
        # rospy.Timer(rospy.Duration(1), self.FSM_callback)

    def reset_pitch(self):
        self.cmd_handle.reset_pitch()
        self.pid_pitch.reset()

    def reset_yaw(self):
        self.cmd_handle.reset_yaw()
        self.pid_yaw.reset()

    def reset_speed(self):
        self.cmd_handle.reset_speed()
        self.pid_forwardSpeed.reset()
        self.pid_sideSpeed.reset()
        self.pid_rotateSpeed.reset()

    def reset_cmd(self):
        # self.reset_pitch()
        self.reset_yaw()
        self.reset_speed()

    def change_FSM_state(self,state):
        rospy.loginfo('change to state: %s',state)
        self.FSM_state_pub.publish('change to state: %s' % state)
        self.FSM_state_enter_first_time = rospy.Time.now()
        self.state = self.FSM_state[state]
        if state == 'REPLAN':
            self.reset_cmd()
            rospy.set_param('enable_replan', True)
        elif state == 'SEEK_TARGET':
            self.seek_curve_x = 0

    def FSM_callback(self,target_uv,distance,laikage_Highstate):
        state = self.state
        change_FSM_state = self.change_FSM_state
        reset_yaw = self.reset_yaw
        reset_speed = self.reset_speed
        reset_cmd = self.reset_cmd
        FSM_state = self.FSM_state
        distance = distance.data
        safe_distance = self.safe_distance
        tmp = 0.2

        if state == FSM_state.INIT:
            reset_cmd()
            change_FSM_state('WAIT_TARGET')

        elif state == FSM_state.WAIT_TARGET:
            if distance == -1:
                if rospy.Time.now().to_nsec() - self.FSM_state_enter_first_time.to_nsec() > 500000000:
                    reset_cmd()
            elif distance <= safe_distance:
                # reset_cmd()
                change_FSM_state('STAND_STILL_TRACK')
            elif distance > safe_distance:
                change_FSM_state('REPLAN')

        elif state == FSM_state.STAND_STILL_TRACK:
            if distance == -1:
                change_FSM_state('SEEK_TARGET')
            elif distance <= safe_distance:
                # make sure the robot has stopped
                if laikage_Highstate.forwardSpeed == 0:
                    self.cmd_handle.cmd.yaw -= self.pid_yaw(target_uv.x)
                    self.cmd_handle.cmd.pitch -= self.pid_pitch(target_uv.y)
                    if self.cmd_handle.cmd.yaw < -0.6 or self.cmd_handle.cmd.yaw > 0.6:
                        self.pid_rotateSpeed.reset()
                        change_FSM_state('STAND_ROTATE_TRACK')
                else:
                    reset_cmd()
            elif distance > self.replan_distance:
                change_FSM_state('REPLAN')

        elif state == FSM_state.STAND_ROTATE_TRACK:
            if distance == -1 or (320 - 0.01*320) < target_uv.x < (320 + 0.01*320):
                reset_cmd()
                change_FSM_state('STAND_STILL_TRACK')
            elif distance > safe_distance:
                change_FSM_state('REPLAN')
            else:
                self.cmd_handle.cmd.mode = 2
                self.cmd_handle.cmd.forwardSpeed = 0
                self.cmd_handle.cmd.rotateSpeed += self.pid_rotateSpeed(target_uv.x)
                self.cmd_handle.cmd.sideSpeed += self.pid_sideSpeed(target_uv.x)   

        elif state == FSM_state.REPLAN:
            # enable_seek_target = rospy.get_param('enable_seek_target')
            if distance == -1:
                rospy.set_param('enable_replan', False)
                change_FSM_state('SEEK_TARGET')
            elif distance <= safe_distance:
                rospy.set_param('enable_replan', False)
                change_FSM_state('STAND_STILL_TRACK')
            else:
                return False

        elif state == FSM_state.SEEK_TARGET:
            if distance == -1:
                self.seek_curve_x += 0.01
                if 0 <= self.seek_curve_x <= 4:
                    self.cmd_handle.cmd.yaw = -0.6*sin(pi*self.seek_curve_x/2)
                if 3 <= self.seek_curve_x <= 5:
                    self.cmd_handle.cmd.pitch = -0.6*cos(pi*self.seek_curve_x/2)
                if 5 <self.seek_curve_x <= 6:
                    self.cmd_handle.cmd.rotateSpeed = 0.1
                if self.seek_curve_x > 6:
                    change_FSM_state('WAIT_TARGET')
            elif distance > safe_distance:
                change_FSM_state('REPLAN')
            elif distance <= safe_distance:
                change_FSM_state('STAND_STILL_TRACK')
            
        else:
            print 'Invalid state!'
        
        # debug
        self.cmd_yaw.publish(self.cmd_handle.cmd.yaw)
        self.cmd_pitch.publish(self.cmd_handle.cmd.pitch)
        self.cmd_rotateSpeed.publish(self.cmd_handle.cmd.rotateSpeed)

        self.cmd_handle.send()

def main(args):
    rospy.init_node('track_fsm', anonymous=False)
    tracker = TrackFsm()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
   
if __name__ == '__main__':
    main(sys.argv)