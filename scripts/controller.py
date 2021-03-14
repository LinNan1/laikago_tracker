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
        self.pos_cmd_pose_pub = rospy.Publisher('/planning/pos_cmd_pose',Odometry,queue_size=10)
        self.laikago_pose_pub = rospy.Publisher('/laikago_tracker/pose',Odometry,queue_size=10)
        self.cmd_handle = laikago_command_handle()

        self.pid_x = PID(2.0, 0.0, 0.02, setpoint=0)
        self.pid_y = PID(2.5, 0.0, 0.025, setpoint=0)
        self.pid_yaw = PID(3.0, 0.0, 0.03, setpoint=0)

        self.pid_x.output_limits = (0,1)
        self.pid_y.output_limits = (-1,1)
        self.pid_yaw.output_limits = (-1,1)

    def call_back(self,pos_cmd, vins_imu):

        pos_cmd_pos = Odometry()
        pos_cmd_pos.pose.pose.position.x = pos_cmd.position.x
        pos_cmd_pos.pose.pose.position.y = pos_cmd.position.y
        q = transformations.quaternion_from_euler(0, 0, pos_cmd.yaw)
        pos_cmd_pos.pose.pose.orientation.x = q[0]
        pos_cmd_pos.pose.pose.orientation.y = q[1]
        pos_cmd_pos.pose.pose.orientation.z = q[2]
        pos_cmd_pos.pose.pose.orientation.w = q[3]
        pos_cmd_pos.header = pos_cmd.header

        self.pos_cmd_pose_pub.publish(pos_cmd_pos)
        
        vins_imu.pose.pose.position.x = vins_imu.pose.pose.position.x - 0.2
        self.laikago_pose_pub.publish(vins_imu)
        
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