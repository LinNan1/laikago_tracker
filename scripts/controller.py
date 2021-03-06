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

        self.safe_distance = rospy.get_param('safe_distance') # m
        self.replan_distance = rospy.get_param('replan_distance') # m

        self.pid_x = PID(1.5, 0.0, 0.01, setpoint=0)
        self.pid_y = PID(1.5, 0.0, 0.023, setpoint=0)
        self.pid_yaw = PID(1.5, 0.0, 0.01, setpoint=0)

        self.pid_x.output_limits = (0,1)
        self.pid_y.output_limits = (-1,1)
        self.pid_yaw.output_limits = (-1,1)

        self.reach_goal_first_time = rospy.Time.now()

    def reset_pid(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_yaw.reset()

    def call_back(self,pos_cmd, vins_imu):

        enable = rospy.get_param('enable_replan')
        # enable = True
        if enable:
            # debug
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
            
            # if pos_cmd.velocity.x == 0.0 and pos_cmd.velocity.y == 0.0:
            #     if rospy.Time.now().to_nsec() - self.reach_goal_first_time.to_nsec() > 500000000:
            #         rospy.set_param('enable_replan', False)
            #         # rospy.set_param('enable_seek_target', True)
            # else:
            self.cmd_handle.cmd.mode = 2
            self.cmd_handle.cmd.forwardSpeed = self.pid_x(vins_imu.pose.pose.position.x - pos_cmd.position.x)
            self.cmd_handle.cmd.sideSpeed = self.pid_y(vins_imu.pose.pose.position.y - pos_cmd.position.y)
            orientation = vins_imu.pose.pose.orientation
            (roll, pitch, yaw) = transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            # (roll, pitch, yaw) = transformations.euler_from_quaternion([
            #     pos_cmd.position.x - vins_imu.pose.pose.position.x, 
            #     pos_cmd.position.y - vins_imu.pose.pose.position.y, 
            #     0, orientation.w])
            self.cmd_handle.cmd.rotateSpeed = self.pid_yaw(yaw - pos_cmd.yaw)

            self.cmd_handle.send()

            self.reach_goal_first_time = rospy.Time.now()
        else:
            self.reset_pid()
        
if __name__ == '__main__':
    rospy.init_node('laikago_controller', anonymous=False)
    traj_controller = TrajController()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")