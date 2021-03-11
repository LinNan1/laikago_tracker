import rospy
from laikago_msgs.msg import HighCmd
from sensor_msgs.msg import Joy
class laikago_command_handle():
    def __init__(self):
        self.high_cmd_pub = rospy.Publisher('/laikago_real/high_cmd',HighCmd,queue_size=10)
        self.joy_sub = rospy.Subscriber('/dji_sdk/rc',Joy,self.joy_call_back)
        self.joy = True
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

    def joy_call_back(self,joy_trigger):
        if joy_trigger.axes[5] != 0:
            self.joy = True
            self.reset()
            if joy_trigger.axes[5] == 3:
                self.cmd.mode = 2
                self.cmd.forwardSpeed = joy_trigger.axes[3]
                self.cmd.rotateSpeed = -joy_trigger.axes[2]
                self.cmd.sideSpeed = -joy_trigger.axes[0]
            else:
                self.cmd.mode = 1
                self.cmd.pitch = joy_trigger.axes[1]
                self.cmd.yaw = joy_trigger.axes[2]
                self.cmd.roll = joy_trigger.axes[0]
                # self.cmd.footRaiseHeight = joy_trigger.axes[1]
                self.cmd.bodyHeight = joy_trigger.axes[3]

            self.high_cmd_pub.publish(self.cmd)
        else:
            self.joy = False
    def send(self):
        if not self.joy:
            self.high_cmd_pub.publish(self.cmd)