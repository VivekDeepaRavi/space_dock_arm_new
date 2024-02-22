#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt32
from std_msgs.msg import Int32


cmd_vel_publisher = None  # Declare cmd_vel_publisher as a global variable

first_position = 0.0
second_position = 0.0
third_position = 0.0

current_flow = ''
i =0


def joint_current_flow(data):    
    global first_position,second_position,third_position,current_flow,i
    current_flow = data.data

    # Extract the position from the joint_state message

    

    # if current_flow == 'id_4_lock_rotate':
    #     print("id_4_lock_rotate")
    #     third_position = 6
    
    # if current_flow == 'id_4_unlock_rotate':
    #     print("id_4_unlock_rotate")
    #     third_position = 0

    joints_str = JointTrajectory()
    joints_str.header = Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names=["first","second","third"]
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(1)
    

    if current_flow == 'magnet_detach':
        pub_magnet_control.publish(0)
        print("magnet_detach")
        pass

    if current_flow == 'attach_to_magnet':
        pub_magnet_control.publish(1)
        print("attach_to_magnet")
        pass

    if current_flow == 'clamp_it':
        first_position = 0.59
        second_position = -0.58
        third_position = 3.14
        point.positions = [first_position,second_position,third_position]
        joints_str.points.append(point)
        pub.publish(joints_str)
        print("Clamped")
    
    if current_flow == 'unclamp_it':
        first_position = -0.44792
        second_position = 0.5522
        third_position = 3.14
        point.positions = [first_position,second_position,third_position]
        joints_str.points.append(point)
        pub.publish(joints_str)
        print("un-Clamped")

    if current_flow == 'id_4_lock_rotate':
        first_position = -0.44792
        second_position = 0.5522
        third_position = 3.14
        point.positions = [first_position,second_position,third_position]
        joints_str.points.append(point)
        pub.publish(joints_str)
        print("id_4_lock_rotate")
    
    if current_flow == 'id_4_unlock_rotate':
        first_position = -0.44792
        second_position = 0.5522
        third_position = 0
        point.positions = [first_position,second_position,third_position]
        joints_str.points.append(point)
        pub.publish(joints_str)
        print("id_4_unlock_rotate")

    print(point.positions)

def callback_ir_data(data):
    if data.data == 4294967295:
        pass





if __name__ == "__main__":
    rospy.init_node("master_control")
    sub_current_flow = rospy.Subscriber('current_flow', String, joint_current_flow)
    sub_ir = rospy.Subscriber('ir_data', UInt32, callback_ir_data)
    pub = rospy.Publisher("/dynamixel_workbench_end_eff/joint_trajectory", JointTrajectory, queue_size=10)
    pub_magnet_control = rospy.Publisher('magnet_control', Int32, queue_size=10)
    # process()
    rospy.spin()

