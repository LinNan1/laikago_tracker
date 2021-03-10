import rospy
from laikago_msgs.msg import HighCmd

class laikago_command:
    def __init__(self):
        self.high_cmd_pub = rospy.Publisher('/laikago_real/high_cmd',HighCmd,queue_size=10)
        self.SendHighROS = HighCmd()
    def reset_rpy(self):
        self.SendHighROS.roll = 0
        self.SendHighROS.yaw = 0
        self.SendHighROS.pitch = 0
    def reset_speed(self):
        self.SendHighROS.mode = 1
        self.SendHighROS.forwardSpeed = 0
        self.SendHighROS.sideSpeed = 0
        self.SendHighROS.rotateSpeed = 0
    def reset(self):
        self.SendHighROS.mode = 1
        self.SendHighROS.forwardSpeed = 0
        self.SendHighROS.sideSpeed = 0
        self.SendHighROS.rotateSpeed = 0
        self.SendHighROS.roll = 0
        self.SendHighROS.yaw = 0
        self.SendHighROS.pitch = 0
    def reset_yaw(self):
        self.SendHighROS.yaw = 0
    def reset_pitch(self):
        self.SendHighROS.pitch = 0
    def set_rpy(roll,pitch,yaw):
        self.SendHighROS.mode = 1
        self.SendHighROS.roll = roll
        self.SendHighROS.pitch = pitch
        self.SendHighROS.yaw = yaw
        if self.SendHighROS.yaw < -1.0: self.SendHighROS.yaw = -1.0
        elif self.SendHighROS.yaw > 1.0: self.SendHighROS.yaw = 1.0
        if self.SendHighROS.pitch < -1.0: self.SendHighROS.pitch = -1.0
        elif self.SendHighROS.pitch > 1.0: self.SendHighROS.pitch = 1.0
        if self.SendHighROS.roll < -1.0: self.SendHighROS.roll = -1.0
        elif self.SendHighROS.roll > 1.0: self.SendHighROS.roll = 1.0
    def set_speed(forwardSpeed,rotateSpeed,sideSpeed):
        self.SendHighROS.mode = 2
        self.SendHighROS.forwardSpeed = forwardSpeed
        self.SendHighROS.sideSpeed = sideSpeed
        self.SendHighROS.rotateSpeed = rotateSpeed

    def send(self):
        self.high_cmd_pub.publish(self.SendHighROS)