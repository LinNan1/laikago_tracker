#!/usr/bin/python
import rospy,message_filters
from quadrotor_msgs.msg import PositionCommand
from nav_msgs.msg import Odometry
from laikago_command_handle import laikago_command_handle
from simple_pid import PID
from tf import transformations
class TrajController:
    def __init__(self):
        self.planning_cmd_sub = message_filters.Subscriber('/planning/pos_cmd',PositionCommand)
        self.vins_imu_sub = message_filters.Subscriber('/vins_estimator/imu_propagate',Odometry)
        self.time_syn = message_filters.ApproximateTimeSynchronizer([self.planning_cmd_sub,self.vins_imu_sub], 10, 0.1, allow_headerless=True)
        self.time_syn.registerCallback(self.call_back)
        self.cmd_handle = laikago_command_handle()

        self.pid_x = PID(1.0, 0.0, 0.0, setpoint=0)
        self.pid_y = PID(1.0, 0.0, 0.0, setpoint=0)
        self.pid_yaw = PID(1.0, 0.0, 0.0, setpoint=0)

        self.pid_x.output_limits = (-1,1)
        self.pid_y.output_limits = (-1,1)
        self.pid_yaw.output_limits = (-1,1)

    def call_back(self,pos_cmd, vins_imu):

        self.cmd_handle.cmd.mode = 2
        self.cmd_handle.cmd.forwardSpeed = self.pid_x(vins_imu.pose.pose.position.x - pos_cmd.position.x)
        self.cmd_handle.cmd.sideSpeed = self.pid_y(vins_imu.pose.pose.position.y - pos_cmd.position.y)
        orientation = vins_imu.pose.pose.orientation
        (roll, pitch, yaw) = transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.cmd_handle.cmd.rotateSpeed = self.pid_yaw(yaw - pos_cmd.yaw)

        self.cmd_handle.send()
        
if __name__ == '__main__':
    rospy.init_node('laikago_controller', anonymous=False)
    traj_controller = TrajController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")