#!/usr/bin/env python2
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from bin_picking.msg import MoveRobotAction, MoveRobotActionGoal, MoveRobotResult
from ur_e_webots.msg import GripperAction, GripperGoal


class MoveItInterfaceNode:
    def __init__(self):
        rospy.init_node("moveit_interface_node", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
        self.arm_client = actionlib.SimpleActionClient("execute_trajectory", moveit_msgs.msg.ExecuteTrajectoryAction)
        self.arm_client.wait_for_server()
        self.action_server = actionlib.SimpleActionServer("bin_picking_moveit_interface", MoveRobotAction, self.run_server, False)
        self.gripper_client = actionlib.SimpleActionClient("gripper_server", GripperAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("Starting action server")
        self.action_server.start()

    def run_server(self, goal):
        if goal.action == "get_image" or goal.action == "get_depth":
            action = None
            if goal.action == "get_image":
                action = "get_image"
            else:
                action = "get_depth"
            gripper_goal = GripperGoal()
            gripper_goal.action = action
            self.gripper_client.send_goal(gripper_goal)
            self.gripper_client.wait_for_result()
            result = self.gripper_client.get_result()
            move_robot_result = MoveRobotResult(success=result.success, rgb_compressed=result.rgb_compressed,
                                                depth_compressed=result.depth_compressed)
            self.action_server.set_succeeded(move_robot_result)
        else:
            try:
                if goal.action == "joint":
                    rospy.loginfo("Moving to joint position")
                    joint_goal = [
                        goal.joint_goal.positions[0],
                        goal.joint_goal.positions[1],
                        goal.joint_goal.positions[2],
                        goal.joint_goal.positions[3],
                        goal.joint_goal.positions[4],
                        goal.joint_goal.positions[5]
                    ]
                    self.arm_group.go(joint_goal, wait=True)
                    self.arm_group.stop()
                    rospy.loginfo("Finished movement")
                elif goal.action == "cartesian":
                    rospy.loginfo("Moving to cartesian position")
                    self.arm_group.set_pose_target(goal.cartesian_goal)
                    plan = self.arm_group.plan()
                    trajectory_goal = moveit_msgs.msg.ExecuteTrajectoryGoal
                    trajectory_goal.trajectory = plan
                    self.arm_client.send_goal(trajectory_goal)
                    self.arm_client.wait_for_result(rospy.Duration(20))
                    rospy.loginfo("Finished movement")
                elif goal.action == "home":
                    rospy.loginfo("Moving to home")
                    self.arm_group.set_named_target("home")
                    plan = self.arm_group.plan()
                    trajectory_goal = moveit_msgs.msg.ExecuteTrajectoryGoal
                    trajectory_goal.trajectory = plan
                    self.arm_client.send_goal(trajectory_goal)
                    self.arm_client.wait_for_result(rospy.Duration(20))
                    rospy.loginfo("Finished movement")
                elif goal.action == "close_gripper":
                    width = goal.gripper_width
                    speed = goal.gripper_speed
                    should_lock = goal.gripper_lock
                    should_grip_box = goal.gripper_grip_box
                    gripper_goal = GripperGoal()
                    gripper_goal.action = "close"
                    gripper_goal.width = width
                    gripper_goal.speed = speed
                    gripper_goal.lock = should_lock
                    gripper_goal.grip_box = should_grip_box
                    self.gripper_client.send_goal(gripper_goal)
                    self.gripper_client.wait_for_result()
                elif goal.action == "open_gripper":
                    gripper_goal = GripperGoal()
                    gripper_goal.action = "open"
                    self.gripper_client.send_goal(gripper_goal)
                    self.gripper_client.wait_for_result()
                elif goal.action == "suction_on":
                    gripper_goal = GripperGoal()
                    gripper_goal.action = "suction_on"
                    self.gripper_client.send_goal(gripper_goal)
                    self.gripper_client.wait_for_result()
                elif goal.action == "suction_off":
                    gripper_goal = GripperGoal()
                    gripper_goal.action = "suction_off"
                    self.gripper_client.send_goal(gripper_goal)
                    self.gripper_client.wait_for_result()
                result = MoveRobotResult(success=True)
                self.action_server.set_succeeded(result=result)
            except Exception as e:
                rospy.logerr(e)
                result = MoveRobotResult(success=False)
                self.action_server.set_succeeded(result=result)


if __name__ == "__main__":
    try:
        node = MoveItInterfaceNode()
    except rospy.ROSInterruptException:
        pass
