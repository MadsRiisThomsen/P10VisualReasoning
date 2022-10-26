#!/usr/bin/env python3
from __future__ import print_function
import sys
import copy
import rospy
import moveit_msgs.msg
import actionlib
from geometry_msgs.msg import Pose
from bin_picking.msg import MoveRobotAction, MoveRobotGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation


class MoveItInterfaceTestNode:
    def __init__(self):
        rospy.init_node("moveit_interface_test_node", anonymous=True)
        self.interface_client = actionlib.SimpleActionClient("bin_picking_moveit_interface", MoveRobotAction)
        self.interface_client.wait_for_server()
        rospy.loginfo("Node started")
        
    def run(self):
        # goal = MoveRobotGoal()
        # goal.action = "joint"
        # goal.joint_goal = JointTrajectoryPoint()
        # goal.joint_goal.positions = [
        #     0,
        #     0,
        #     0,
        #     0,
        #     0,
        #     0
        # ]
        # self.interface_client.send_goal(goal)
        # self.interface_client.wait_for_result(rospy.Duration(10))

        # rospy.loginfo("Sending cartesian goal")
        # goal.action = "cartesian"
        # goal.cartesian_goal = Pose()
        # coordinates = [0.373, -0.0165, 0.257, 2.405, 1.018, 2.52]
        # rotation = Rotation.from_rotvec(coordinates[3:]).as_quat()
        # goal.cartesian_goal.position.x = coordinates[0]
        # goal.cartesian_goal.position.y = coordinates[1]
        # goal.cartesian_goal.position.z = coordinates[2]
        # goal.cartesian_goal.orientation.x = rotation[0]
        # goal.cartesian_goal.orientation.y = rotation[1]
        # goal.cartesian_goal.orientation.z = rotation[2]
        # goal.cartesian_goal.orientation.w = rotation[3]
        # self.interface_client.send_goal(goal)
        # self.interface_client.wait_for_result(rospy.Duration(10))

        rospy.loginfo("Send move to home goal")
        goal = MoveRobotGoal()
        goal.action = "home"
        self.interface_client.send_goal(goal)
        self.interface_client.wait_for_result(rospy.Duration(10))

        rospy.loginfo("Finished")


if __name__ == "__main__":
    try:
        node = MoveItInterfaceTestNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
