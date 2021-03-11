#!/usr/bin/python
import rospy
from quadrotor_msgs.msg import PositionCommand
from laikago_command_handle import laikago_command_handle
class TrajServer:
    def __init__(self):
        self.planning_cmd_sub = rospy.Subscriber('/planning/pos_cmd',PositionCommand, self.call_back)
        self.cmd_handle = laikago_command_handle()
    def call_back(self,pos_cmd):
        self.cmd_handle.set_speed(pos_cmd.velocity.x,-pos_cmd.yaw_dot,pos_cmd.velocity.y)
        self.cmd_handle.send()
if __name__ == '__main__':
    rospy.init_node('laikago_controller', anonymous=False)
    traj_server = TrajServer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")