#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input
import time
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import Float32
from std_msgs.msg import String
from sensor_msgs.msg import JointState


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information

        #move_group.allowLooking(True)
        move_group.allow_replanning(True)
        move_group.set_num_planning_attempts(20)
        move_group.set_planner_id("RRTstar")
        move_group.set_planning_time(5)


        #move_group.set_planner_id('RRT')
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        #print("============ Planning frame: %s" % planning_frame)
        #print(move_group.get_planner_id) 

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        #print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        #print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        #print("============ Printing robot state")
        #print(robot.get_current_state())
        #print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self,j1e,j2e,j3e):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = j1e 
        if j2e > 0:
            joint_goal[1] = j2e
        else:
            joint_goal[1] = j2e +6.28
        joint_goal[2] = j3e

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def get_current_State(self):
        move_group = self.move_group
        current_pose1 = move_group.get_current_pose()

        # Extract X, Y, and Z coordinates
        x = current_pose1.pose.position.x
        y = current_pose1.pose.position.y
        z = current_pose1.pose.position.z
        w = current_pose1.pose.orientation.w
        xo = current_pose1.pose.orientation.x
        yo = current_pose1.pose.orientation.y
        zo = current_pose1.pose.orientation.z


        # Print the coordinates
        print("X:", x)
        print("Y:", y)
        print("Z:", z)
        print("W:", w)
        print("xo:", xo)
        print("yo:", yo)
        print("zo:", zo)

    def go_to_pose_goal(self,xe,ye,ze):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()

        # pose_goal.position.x = -0.018545570158773138
        # pose_goal.position.y = 0.2726520661515016
        # pose_goal.position.z = 0.1859999999999987

        # pose_goal.position.x = 0.09893309800498006
        # pose_goal.position.y = 0.0004626299379448125
        # pose_goal.position.z = 0.1859999999999987

        pose_goal.position.x = xe
        pose_goal.position.y = ye
        pose_goal.position.z = ze


        ####################################################

        # pose_goal.position.x =   0.55769
        # pose_goal.position.y = 0.0
        # pose_goal.position.z = 0.18

        # pose_goal.position.x =  0.18233182276048346
        # pose_goal.position.y = -0.2387184167550397
        # pose_goal.position.z = 0.18

        # pose_goal.position.x = 0.370503
        # pose_goal.position.y = 0.061114
        # pose_goal.position.z = 0.18


        # pose_goal.position.x =  0.25815807641143623
        # pose_goal.position.y = 0.2624835542278256
        # pose_goal.position.z = 0.18

        #pose_goal.orientation.w =  0.9999999985017632
        #pose_goal.orientation.x = -2.48397593503616e-07
        #pose_goal.orientation.y = 1.7497356521994868e-11
        #pose_goal.orientation.z = -5.4739490705967714e-05


        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return success

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


operation = True    
detached =0
start_time =0
end_time= 0
initial =0
i =0
obj_posi = 100
yy_obj_loc =0.0   
button_data = 'null'
move_to_motor_posi = True
motor_posi = []
current_flow =''
present_position =''    


def main():
    global initial,end_time,start_time,i,detached,yy_obj_loc
    global obj_posi, button_data, move_to_motor_posi,motor_posi,current_flow,present_position

    pub = rospy.Publisher('current_flow', String, queue_size=1)
    

    try:
        while operation:
            tutorial = MoveGroupPythonInterfaceTutorial()       
            #print(obj_posi)

            if button_data == 'start':
                button_data = 'initiated'
                print("startedddd")
                initial =0

            if move_to_motor_posi == True:
                current_flow = 'attach_to_magnet'
                pub.publish(current_flow)
                time.sleep(5)
                tutorial.go_to_joint_state(motor_posi[0], motor_posi[1], motor_posi[2])
                move_to_motor_posi = False
                time.sleep(5)
                current_flow = 'fk_done'
                pub.publish(current_flow)
            else:                
                if (initial ==0 and detached ==0):
                    print("Home position")
                    #aa = tutorial.go_to_pose_goal(0.1084020652,-0.002778003778,0.18599999999)
                    aa = tutorial.go_to_pose_goal(-0.13,0.0,0.18599999999)
                    while aa == False:
                        aa = tutorial.go_to_pose_goal(-0.13,0.0,0.18599999999)
                        #aa =tutorial.go_to_pose_goal(0.1084020652,-0.002778003778,0.18599999999)
                    if aa == True:
                        print('Reached- Home-posi')
                    time.sleep(5)
                    present_position = 'home'
                    #time.sleep(2) 
                    initial =1
                    i=0
                elif (initial ==1):
                    print("Left Home position")
                    aa =tutorial.go_to_pose_goal(-0.13,0.1,0.18599999999)
                    #aa =tutorial.go_to_pose_goal(0.10,0.0,0.18599999999)
                    while aa == False:
                        #aa =tutorial.go_to_pose_goal(0.10,0.0,0.18599999999)
                        aa = tutorial.go_to_pose_goal(-0.13,0.1,0.18599999999)
                    
                    if aa == True:
                        print('Reached- left-Home-posi')
                    time.sleep(5)
                    present_position = 'left_home'
                    initial =2
                    end_time = time.time()

                elif (initial ==2):
                    print(start_time-end_time)
                    if (i==0):
                        start_time =time.time()
                        i==1
                    if(start_time-end_time < 10):
                        time.sleep(5)
                        print(obj_posi)
                        if(obj_posi < 10.1):
                            print("Goal position is in left")
                            if (obj_posi >= 0):
                                yy_obj_loc = 0.1 - (obj_posi/100)
                            else:
                                yy_obj_loc = 0.1 - (obj_posi/100)

                            print("the goal point is:%f", yy_obj_loc)

                            print('Reached- left-goal3333-Home-posi')
                            aa = tutorial.go_to_pose_goal(0.22,yy_obj_loc,0.18599999999)
                            while aa == False:
                                aa = tutorial.go_to_pose_goal(0.22,yy_obj_loc,0.18599999999)
                            if aa == True:
                                print('Reached- left-goal3333-Home-posi')
                            time.sleep(15)


                            print('Reached- left-goal4444-Home-posi')
                            aa = tutorial.go_to_pose_goal(0.14,yy_obj_loc,0.18599999999)
                            while aa == False:
                                aa = tutorial.go_to_pose_goal(0.14,yy_obj_loc,0.18599999999)
                            if aa == True:
                                print('Reached- left-goal4444-Home-posi')
                            time.sleep(5)

                            #time.sleep(10)
                            current_flow = 'magnet_detach'
                            pub.publish(current_flow)
                            time.sleep(5)
                            current_flow = 'id_4_lock_rotate'
                            pub.publish(current_flow)
                            time.sleep(5)
                            current_flow = 'clamp_it'
                            pub.publish(current_flow)

                            initial = 5
                            detached =1
                        else:
                            pass
                    else:
                        print("not detected in left side")
                        print("Right Home position")
                        aa = tutorial.go_to_pose_goal(-0.13,-0.1,0.18599999999)
                        while aa == False:
                            aa = tutorial.go_to_pose_goal(-0.13,-0.1,0.18599999999)
                        if aa == True:
                            print('Reached- right-posi')
                        time.sleep(10)
                        present_position = 'right_home'
                        end_time = time.time()
                        start_time =0
                        initial =3
                        i=0
                
                elif (initial ==3):
                    print(start_time-end_time)
                    if (i==0):
                        start_time =time.time()
                        i==1
                    if(start_time-end_time < 10):
                        print(obj_posi)
                        if(obj_posi<10.1):
                            print("Goal position is in right")
                            if (obj_posi >=0):
                                yy_obj_loc = -0.1 - (obj_posi/100)
                            else:
                                yy_obj_loc = -0.1 - (obj_posi/100)
                            aa =tutorial.go_to_pose_goal(0.20,yy_obj_loc,0.18599999999)
                            while aa == False:
                                aa = tutorial.go_to_pose_goal(0.20,yy_obj_loc,0.18599999999)

                            if aa == True:
                                print('Reached- right-Home-posi')

                            time.sleep(10)

                            current_flow = 'magnet_detach'
                            pub.publish(current_flow)
                            time.sleep(5)
                            current_flow = 'id_4_lock_rotate'
                            pub.publish(current_flow)
                            time.sleep(5)
                            current_flow = 'clamp_it'
                            pub.publish(current_flow)

                            initial =5
                            detached =1
                        else:
                            pass
                    else:
                        print("not detected in right side")
                        initial=4
                        print("Home position")
                        aa = tutorial.go_to_pose_goal(0.1084020652,-0.002778003778,0.18599999999)
                        while aa == False:
                            aa = tutorial.go_to_pose_goal(0.1084020652,-0.002778003778,0.18599999999)
                        if aa == True:
                            print('Reached- Home-posi111111')
                        time.sleep(10)
                        present_position = 'home'

                elif (initial ==5):
                    if button_data == 'detach':
                        current_flow = 'unclamp_it'
                        pub.publish(current_flow)
                        time.sleep(5)
                        
                        current_flow = 'id_4_unlock_rotate'
                        pub.publish(current_flow)

                        current_flow = 'attach_to_magnet'
                        pub.publish(current_flow)
                        time.sleep(5)

                        initial = 6
                        detached = 0

                        print("Home position")
                        tutorial.go_to_pose_goal(0.1084020652,-0.002778003778,0.18599999999)
                        while aa == False:
                            aa = tutorial.go_to_pose_goal(0.1084020652,-0.002778003778,0.18599999999)
                        if aa == True:
                            print('Reached- Home-posi2222222')
                        
                        time.sleep(10)
                        present_position = 'home'

            #tutorial.go_to_joint_state(x1,y1,z1)
            #tutorial.go_to_joint_state(x1,y1,z1)


            #print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
def callback_motor_posi(data):
    global motor_posi
    motor_posi = data.position
    #print(motor_posi)
    pass

def callback_button(data):
    global button_data
    button_data = data.data
    print(button_data)


def callback_posi(data):
    global obj_posi
    obj_posi= round(data.data,4)

if __name__ == "__main__":
    rospy.Subscriber("object_position", Float32, callback_posi)
    rospy.Subscriber("button", String, callback_button)
    rospy.Subscriber("dynamixel_workbench/joint_states", JointState, callback_motor_posi)

    main()