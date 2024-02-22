#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


cmd_vel_publisher = None  # Declare cmd_vel_publisher as a global variable

first_position = 0.0
second_position = 0.0
third_position = 0.0



def joint_state_callback(joint_state):
    global first_position 
    global second_position 
    global third_position 
    # Extract the position from the joint_state message

    
    positions = joint_state.position

    first_position = positions[1]
    # if positions[0] < -3.14159:
    #     first_position = positions[0] + 6.28319
    # else:
    #     first_position = positions[0]

    # if positions[1] < -3.14159:
    #     second_position = positions[1] + 6.28319
    # else:
    #     second_position = positions[1]

    # third_position = positions[2]
    
    # Process the extracted data as needed
    # For example, you can store them in variables or perform computations
    # Print the extracted data
    # print("Positions:", positions[0])
    # print("Positions:", positions[1])
    # print("Positions:", positions[2])

# def process():
    
#     global first_position 
#     global second_position 
#     global third_position 

    joints_str = JointTrajectory()
    joints_str.header = Header()
    joints_str.header.stamp = rospy.Time.now()
    joints_str.joint_names=["first"]
    point = JointTrajectoryPoint()
    point.positions = [first_position]
    print(point.positions)
    point.time_from_start = rospy.Duration(1)
    joints_str.points.append(point)
    pub.publish(joints_str)

    process_data = ""

if __name__ == "__main__":
    rospy.init_node("test")
    sub = rospy.Subscriber('joint_states', JointState, joint_state_callback)
    pub = rospy.Publisher("/dynamixel_workbench/joint_trajectory", JointTrajectory, queue_size=1)
    # process()
    rospy.spin()

