#!/usr/bin/env python
import time
import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import trajectory_msgs.msg
from std_msgs.msg import String

import copy
import actionlib
import roslib
import rospkg
import numpy as np

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Move base using navigation stack
class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.client.wait_for_server()

    def goto(self, x, y, theta, frame="map"):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = x
        move_goal.target_pose.pose.position.y = y
        move_goal.target_pose.pose.orientation.z = sin(theta/2.0)
        move_goal.target_pose.pose.orientation.w = cos(theta/2.0)
        move_goal.target_pose.header.frame_id = frame
        move_goal.target_pose.header.stamp = rospy.Time.now()

        # TODO wait for things to work
        self.client.send_goal(move_goal)
        self.client.wait_for_result()

class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        result = self.client.wait_for_result()

        return result

class GripperActionClient(object):
    def __init__(self):

        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient(
            "gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")

    def move_gripper(self, gripper_x, max_effort, timeout=5.0):

        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = max_effort
        gripper_goal.command.position = gripper_x

        self.gripper_client.send_goal(gripper_goal)
        result = self.gripper_client.wait_for_result(rospy.Duration(timeout))

        return result


class MoveFetch(object):

    def __init__(self):

        rospy.loginfo("In Move Fetch Calss init...")

        # Init Torso Action
        self.torso_action = FollowTrajectoryClient(
            "torso_controller", ["torso_lift_joint"])

        # Gripper Action
        self.gripper_action = GripperActionClient()

        # Point Head action
        self.head_action = PointHeadClient()

        # MOVEIT
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.logdebug("moveit_commander initialised...")

        rospy.logdebug("Starting Robot Commander...")
        self.robot = moveit_commander.RobotCommander()
        rospy.logdebug("Starting Robot Commander...DONE")

        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.logdebug("PlanningSceneInterface initialised...DONE")
        self.group = moveit_commander.MoveGroupCommander("arm")
        rospy.logdebug("MoveGroupCommander for arm initialised...DONE")

        rospy.logwarn("self.group TYPE==>"+str(type(self.group)))

        rospy.loginfo("FETCH ready to move!")

    def move_manager(self, pose_requested, joints_array_requested, movement_type_requested):

        success = False

        if movement_type_requested == "TCP":
            success = self.ee_traj(pose_requested)
        elif movement_type_requested == "JOINTS":
            success = self.joint_traj(joints_array_requested)
        elif movement_type_requested == "TORSO":
            torso_height = joints_array_requested[0]
            success = self.move_torso(torso_height)
        elif movement_type_requested == "HEAD":
            XYZ = [joints_array_requested[0],
                   joints_array_requested[1],
                   joints_array_requested[2]]
            success = self.move_head_point(XYZ)
        elif movement_type_requested == "GRIPPER":
            gripper_x = joints_array_requested[0]
            max_effort = joints_array_requested[1]

            success = self.move_gripper(gripper_x, max_effort)
        else:
            rospy.logerr("Asked for non supported movement type==>" +
                         str(movement_type_requested))

        return success

    def ee_traj(self, pose):

        pose_frame = self.group.get_pose_reference_frame()

        if pose_frame != "base_link":
            new_reference_frame = "base_link"
            self.group.set_pose_reference_frame(new_reference_frame)

            pose_frame = self.group.get_pose_reference_frame()

        else:
            pass

        self.group.set_pose_target(pose)

        result = self.execute_trajectory()

        return result

    def joint_traj(self, positions_array):

        self.group_variable_values = self.group.get_current_joint_values()
        rospy.logdebug("Group Vars:")
        rospy.logdebug(self.group_variable_values)
        rospy.logdebug("Point:")
        rospy.logdebug(positions_array)
        self.group_variable_values[0] = positions_array[0]
        self.group_variable_values[1] = positions_array[1]
        self.group_variable_values[2] = positions_array[2]
        self.group_variable_values[3] = positions_array[3]
        self.group_variable_values[4] = positions_array[4]
        self.group_variable_values[5] = positions_array[5]
        self.group_variable_values[6] = positions_array[6]
        self.group.set_joint_value_target(self.group_variable_values)
        result = self.execute_trajectory()

        return result

    def move_torso(self, torso_height):
        """
        Changes the Fetch Torso height based on the height given.
        """
        ## OUR Changes
        result = self.torso_action.move_to([torso_height, ])

        return result

    def move_gripper(self, gripper_x, max_effort):
        """
        Moves the gripper to given pose
        """
        result = self.gripper_action.move_gripper(gripper_x, max_effort)

        return result

    def move_head_point(self, XYZ, frame="base_link"):
        """
        Changes the Fetch Torso height based on the height given.
        """
        result = self.head_action.look_at(XYZ[0], XYZ[1], XYZ[2], frame)

        return result

    def execute_trajectory(self):

        self.plan = self.group.plan()
        result = self.group.go(wait=True)

        return result

    def ee_pose(self):

        gripper_pose = self.group.get_current_pose()

        rospy.logdebug("EE POSE==>"+str(gripper_pose))

        return gripper_pose

    def ee_rpy(self, request):

        gripper_rpy = self.group.get_current_rpy()

        return gripper_rpy
def friction_base_tests():
    arm_joint_positions_L = [-1.57, -0.9, 0, 0.9, 0.0, 1.57, 0.0]
    arm_joint_positions_R = [1.57, -0.9, 0, 0.9, 0.0, 1.57, 0.0]
    move_fetch_obj = MoveFetch()

    side = "LEFT"

    rate_obj = rospy.Rate(1)
    while not rospy.is_shutdown():

        if side == "LEFT":
            side = "RIGHT"
            move_fetch_obj.move_manager(pose_requested=None,
                                        joints_array_requested=arm_joint_positions_L,
                                        movement_type_requested="JOINTS")
        else:
            side = "LEFT"
            move_fetch_obj.move_manager(pose_requested=None,
                                        joints_array_requested=arm_joint_positions_R,
                                        movement_type_requested="JOINTS")
        rate_obj.sleep()
def send_Coords():
    # move robot to coordinates
    move_base = MoveBaseClient()
    #rospy.loginfo("Moving to table...")
    move_base.goto(2.250, 3.118, 0.0)
    move_base.goto(2.750, 3.118, 0.0)
def gripper_test():

    move_fetch_obj = MoveFetch()
    grip_position = 0.5
    max_effort = 10.0
    move_fetch_obj.move_manager(pose_requested=[],
                                joints_array_requested=[grip_position, max_effort],
                                movement_type_requested="GRIPPER")

    move_fetch_obj.move_manager(pose_requested=[],
                                joints_array_requested=[0, max_effort],
                                movement_type_requested="GRIPPER")
def torso_test():

    move_fetch_obj = MoveFetch()


    # move torso up to height 0.5, back down to 0, and back up to 0.5
    move_fetch_obj.move_manager(pose_requested=[],
                                joints_array_requested=[0.5],
                                movement_type_requested="TORSO")
    move_fetch_obj.move_manager(pose_requested=[],
                                joints_array_requested=[0],
                                movement_type_requested="TORSO")
    move_fetch_obj.move_manager(pose_requested=[],
                                joints_array_requested=[0.5],
                                movement_type_requested="TORSO")

def move_tests():

    move_fetch_obj = MoveFetch()

    # Move P1

    move_fetch_obj.move_manager(pose_requested=[],
                                joints_array_requested=[0.6, 0.0, 0.75],
                                movement_type_requested="HEAD")

    pose = Pose()

    pose.position.x = 0.6
    pose.position.y = 0.1
    pose.position.z = 1.0

    quat = quaternion_from_euler(ai=0.0, aj=1.57, ak=0.0)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    LEFT_Y = -0.09
    RIGHT_Y = 0.09
    UP_Z = 1.0
    DOWN_Z = 0.9

    pose_y_seq = [RIGHT_Y, RIGHT_Y, RIGHT_Y,
                  LEFT_Y, LEFT_Y, LEFT_Y, LEFT_Y, LEFT_Y, RIGHT_Y, RIGHT_Y]
    pose_z_seq = [UP_Z, DOWN_Z, UP_Z, UP_Z,
                  DOWN_Z, UP_Z, DOWN_Z, UP_Z, UP_Z, DOWN_Z]
    seq_num = 0
    raw_input("Start...Go Right")
    while not rospy.is_shutdown():
        print("Seq Num="+str(seq_num))

        pose.position.y = pose_y_seq[seq_num]
        pose.position.z = pose_z_seq[seq_num]
        print(str(pose.position))
        move_fetch_obj.move_manager(pose_requested=pose,
                                    joints_array_requested=[],
                                    movement_type_requested="TCP")

        if (seq_num == 1 or seq_num == 6):
            print("CLOSE GRIPPER")
            max_effort = 10.0
            grip_position = 0.02
            move_fetch_obj.move_manager(pose_requested=None,
                                        joints_array_requested=[
                                            grip_position, max_effort],
                                        movement_type_requested="GRIPPER")
        elif(seq_num == 4 or seq_num == 9):
            print("OPEN GRIPPER")
            max_effort = 10.0
            grip_position = 0.5
            move_fetch_obj.move_manager(pose_requested=None,
                                        joints_array_requested=[
                                            grip_position, max_effort],
                                        movement_type_requested="GRIPPER")

        seq_num += 1
        if seq_num > 9:
            seq_num = 0

def reset_all_the_things():

    move_base = MoveBaseClient()
    #rospy.loginfo("Moving to table...")
    move_base.goto(0, 0, 0)

    # LOOK UP FETCH WEBSITE BOOKMARK RESETTING ROBOT STUFF
    # maybe save coordinates of inital pose so that robot can get back to it? (robot bad at locating itself)
    # TODO: reset head
    # TODO: reset gripper
    # TODO: reset torso
    # TODO: tuck arm


# if __name__ == '__main__':
#
#     #print('no robot for you today')
#     #sys.exit(0)
#
#     rospy.init_node('fetch_move_node', anonymous=True, log_level=rospy.DEBUG)
#
#     gripper_test()
#     torso_test()
#     send_Coords()
#     #move_tests()
#     # friction_base_tests()
#
#     reset_all_the_things()
###############################################
CONTROL_PERIOD = rospy.Duration(1)

class Controller:
    # called when an object of type Controller is created
    def __init__(self):
        # initialize rospy'
        rospy.init_node('starter')

        # set up publisher for commanded velocity
        # controller node directly interacts with cmd_vel, which then interacts with fetch robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                           Twist, queue_size=1)
        self.torso_sub = rospy.Subscriber('/torso_sub', Twist, self.torso_callback)

        self.state = 'forward'
        print("INIT self.state: ", self.state)

        # set up control timer at 100 Hz
        self.timer = rospy.Timer(CONTROL_PERIOD, self.control_callback)
        print("timer set up")

        self.move_fetch_obj = MoveFetch()



    # called 100 times per second
    def control_callback(self, timer):

        # initialize commanded vel to 0, 0
        cmd_vel = Twist()

        # state maps straightforwardly to command
        print("CC self.state: ", self.state)
        if self.state == 'backward':
            cmd_vel.linear.x = -0.3
        elif self.state == 'forward':
            cmd_vel.linear.x = 0.3
        elif self.state == 'turn_left':
            cmd_vel.angular.z = 1.5
        elif self.state == 'turn_right':
            cmd_vel.angular.z = -1.5
        # elif self.state == 'move_to'
        # else: # forward
        #     cmd_vel = self.wander_action
        self.cmd_vel_pub.publish(cmd_vel)

    # called by main function below (after init)
    def torso_callback(self, data):
        # move torso up to height 0.5, back down to 0, and back up to 0.5
        self.move_fetch_obj.move_manager(pose_requested=[],
                                    joints_array_requested=[data.linear.x],
                                    movement_type_requested="TORSO")
        print("move torso: ",data)
    def run(self):

        # timers and callbacks are already set up, so just spin.
        # if spin returns we were interrupted by Ctrl+C or shutdown
        rospy.spin()


# main function
if __name__ == '__main__':
    try:
        print("start\n")
        ctrl = Controller()
        ctrl.run()
    except rospy.ROSInterruptException:
        print("interrupt")
        pass
