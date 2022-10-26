#!/usr/bin/env python2
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import json
import geometry_msgs


def moveit_test():
    rospy.init_node("moveit_test", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    arm_client = actionlib.SimpleActionClient("execute_trajectory", moveit_msgs.msg.ExecuteTrajectoryAction)
    arm_client.wait_for_server()
    arm_group.set_named_target("up")
    plan = arm_group.plan()
    goal = moveit_msgs.msg.ExecuteTrajectoryGoal
    goal.trajectory = plan
    print("sending goal")
    #pub.publish(goal)
    arm_client.send_goal(goal)
    print("sent goal")
    arm_client.wait_for_result(rospy.Duration.from_sec(5.0))
    result = arm_client.get_result()
    print("done")
    print(result)


if __name__ == "__main__":
    try:
        moveit_test()
    except rospy.ROSInterruptException:
        pass
