#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


cmd_vel_publisher = None  # Declare cmd_vel_publisher as a global variable

first_position = 0.0
second_position = 0.0
third_position = 0.0

current_flow = ''

def joint_state_callback(joint_state):
    global first_position,second_position,third_position,current_flow

    # Extract the position from the joint_state message
    positions = joint_state.position
    first_position = positions[0]
    second_position = positions[1]
    third_position = positions[2]# Process the extracted data as needed
    

    joints_str = JointTrajectory()
    joints_str.header = Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names=["first","second","third"]
    point = JointTrajectoryPoint()
    point.positions = [first_position,second_position,third_position]
    # print(point.positions)
    point.time_from_start = rospy.Duration(1)
    joints_str.points.append(point)

    if current_flow == 'fk_done':
        pub.publish(joints_str)

    process_data = ""


def joint_current_flow(data):
    global current_flow
    current_flow = data.data
    if current_flow=='magnet_detach':
        magpub.publish(0)
        print("magnet off")
    elif current_flow=='attach_to_magnet':
        magpub.publish(1)
        print("magnet on")
    elif current_flow=='id_4_lock_rotate':
        joints_str = JointTrajectory()
        joints_str.header = Header()
        joints_str.header.stamp = rospy.Time.now()
        joints_str.joint_names=["fifth"]
        point = JointTrajectoryPoint()
        point.positions = [-0.5]
        point.time_from_start = rospy.Duration(1)
        joints_str.points.append(point)
        pub1.publish(joints_str)
        print("lock rotation done")
    elif current_flow=='id_4_unlock_rotate':
        joints_str = JointTrajectory()
        joints_str.header = Header()
        joints_str.header.stamp = rospy.Time.now()
        joints_str.joint_names=["fifth"]
        point = JointTrajectoryPoint()
        point.positions = [-3.14]
        point.time_from_start = rospy.Duration(1)
        joints_str.points.append(point)
        pub1.publish(joints_str)
        print("unlock rotation done")
    elif current_flow=='clamp_it':
        joints_str = JointTrajectory()
        joints_str.header = Header()
        joints_str.header.stamp = rospy.Time.now()
        joints_str.joint_names=["fourth","sixth"]
        point = JointTrajectoryPoint()
        point.positions = [1.27,2.14]
        point.time_from_start = rospy.Duration(1)
        joints_str.points.append(point)
        pub1.publish(joints_str)
        print("clamping done")
    elif current_flow=='unclamp_it':
        joints_str = JointTrajectory()
        joints_str.header = Header()
        joints_str.header.stamp = rospy.Time.now()
        joints_str.joint_names=["fourth","sixth"]
        point = JointTrajectoryPoint()
        point.positions = [0.0859,3.0793]
        point.time_from_start = rospy.Duration(1)
        joints_str.points.append(point)
        pub1.publish(joints_str)
        print("unclamping done")


if __name__ == "__main__":
    rospy.init_node("test")
    sub = rospy.Subscriber('joint_states', JointState, joint_state_callback)
    sub_current_flow = rospy.Subscriber('current_flow', String, joint_current_flow)
    pub = rospy.Publisher("/dynamixel_workbench/joint_trajectory", JointTrajectory, queue_size=1)
    pub1 = rospy.Publisher("/dynamixel_workbench_1/joint_trajectory", JointTrajectory, queue_size=1)
    magpub=rospy.Publisher("/magnet_control",Int32,queue_size=1)
    rospy.spin()

