import rospy
from laikago_msgs.msg import HighCmd

class laikago_command_handle():
    def __init__(self):
        self.high_cmd_pub = rospy.Publisher('/laikago_real/high_cmd',HighCmd,queue_size=10)
        self.cmd = HighCmd()
    def reset_rpy(self):
        self.cmd.roll = 0
        self.cmd.yaw = 0
        self.cmd.pitch = 0
    def reset_speed(self):
        self.cmd.mode = 1
        self.cmd.forwardSpeed = 0
        self.cmd.sideSpeed = 0
        self.cmd.rotateSpeed = 0
    def reset(self):
        self.cmd.mode = 1
        self.cmd.forwardSpeed = 0
        self.cmd.sideSpeed = 0
        self.cmd.rotateSpeed = 0
        self.cmd.roll = 0
        self.cmd.yaw = 0
        self.cmd.pitch = 0
    def reset_yaw(self):
        self.cmd.yaw = 0
    def reset_pitch(self):
        self.cmd.pitch = 0
    def set_rpy(self,roll,pitch,yaw):
        self.cmd.mode = 1
        self.cmd.roll = roll
        self.cmd.pitch = pitch
        self.cmd.yaw = yaw
        if self.cmd.yaw < -1.0: self.cmd.yaw = -1.0
        elif self.cmd.yaw > 1.0: self.cmd.yaw = 1.0
        if self.cmd.pitch < -1.0: self.cmd.pitch = -1.0
        elif self.cmd.pitch > 1.0: self.cmd.pitch = 1.0
        if self.cmd.roll < -1.0: self.cmd.roll = -1.0
        elif self.cmd.roll > 1.0: self.cmd.roll = 1.0
    def set_speed(self,forwardSpeed,rotateSpeed,sideSpeed):
        self.cmd.mode = 2
        self.cmd.forwardSpeed = forwardSpeed
        self.cmd.sideSpeed = sideSpeed
        self.cmd.rotateSpeed = rotateSpeed

    def send(self):
        self.high_cmd_pub.publish(self.cmd)