#!/usr/bin/env python3
from unittest import result
from vision_lib.vision_controller import ObjectInfo
import rospy
import actionlib
import cv2
from cv_bridge import CvBridge
import numpy as np
import cv_bridge


##P10 UPDATES START

import moveit_msgs
import geometry_msgs
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotTrajectory



import fhMoveitUtils.moveit_utils as moveit

"""
from fh_moveit_service.srv import moveitMoveToNamedSrv, moveitMoveToNamedSrvResponse
from fh_moveit_service.srv import moveitPlanToNamedSrv, moveitPlanToNamedSrvResponse
from fh_moveit_service.srv import moveitPlanFromPoseToPoseSrv, moveitPlanFromPoseToPoseSrvResponse
from fh_moveit_service.srv import moveitMoveToPoseSrv, moveitMoveToPoseSrvResponse
from fh_moveit_service.srv import moveitExecuteSrv, moveitExecuteSrvResponse
from fh_moveit_service.srv import moveitRobotStateSrv, moveitRobotStateSrvResponse
from fh_moveit_service.srv import moveitPlanToPoseSrv, moveitPlanToPoseSrvResponse
from fh_moveit_service.srv import moveitGetJointPositionAtNamed, moveitGetJointPositionAtNamedResponse
from fh_moveit_service.srv import moveitGripperCloseSrv, moveitGripperCloseSrvResponse
from fh_moveit_service.srv import moveitGripperOpenSrv, moveitGripperOpenSrvResponse

from fh_moveit_service import moveitMoveToNamedSrv
"""

from fh_moveit_service.srv import *

##P10 UPDATES END



class RobotController:
    def __init__(self):
        self.bridge = CvBridge()
        self.is_home = True

        moveit.setMaxVelocityScalingFactor(0.1)
        moveit.setMaxAcceleratoinScalingFactor(0.2)
        moveit.setPlanningTime(1.0)
        moveit.setNumPlanningAttempts(25)


    def pick_up(self, object_info: ObjectInfo, rgb, object_location):
        rospy.loginfo("Waiting for server")

        if self.is_home == False:
            moveit.moveToNamed("camera_ready_1")
        moveit.gripperOpen()

        if object_location == "white cover":
            moveit.moveToNamed("left_fixture")
        elif object_location == "pcb":
            moveit.moveToNamed("middle_fixture")
        elif object_location == "blue cover":
            moveit.moveToNamed("right_fixture")
        
        moveit.gripperClose()
        moveit.moveToNamed("camera_ready_1")

        return True

        """
        self.client.wait_for_server()

        goal = PickObjectGoal()
        goal.mask = self.bridge.cv2_to_imgmsg(object_info.mask_full)
        goal.reference_img = self.bridge.cv2_to_imgmsg(rgb)
        goal.depth_img = self.bridge.cv2_to_imgmsg(depth)
        goal.command = "pick_object"

        rospy.loginfo("sending goal")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()

        return result.success
        """

    def place(self, position, rgb):
        rospy.loginfo("Waiting for server")

        if self.is_home == False:
            moveit.moveToNamed("camera_ready_1")
        moveit.moveToNamed("assembly_location")
        moveit.gripperOpen()

        return True

        """
        self.client.wait_for_server()

        goal = PickObjectGoal()
        goal.command = "place_object"
        goal.place_image_x = round(position[0])
        goal.place_image_y = round(position[1])
        goal.place_world_z = position[2]
        goal.reference_img = self.bridge.cv2_to_imgmsg(rgb)

        rospy.loginfo("sending goal")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result.success
        """

    def point_at(self, object_info: ObjectInfo, rgb):
        """
        rospy.loginfo("Using point service")

        moveit.pointToNamed("named of fixture that matches with the detected object")

        Thinking that ObjectInfo should have a variable for which fixture.
        The camera could use classical cv to compute which fixture, no need for CNN for that
        Ex: Find red cover, red cover is in fixture 1,
        from the setup and the camera pos we know where in the image fixture 1 would be,
        so we look for red cover in an image, detect the pixel corrds, pixel coords will show what fixture it is


        have to use moveit.moveToPose(req) for cartesian coordinates
        req needs req.pose which is geometry_msgs.pose




        return result.success
        """
        rospy.loginfo("Deprecated function, dont use it")
        return True


        rospy.loginfo("Waiting for server")
        self.client.wait_for_server()

        goal = PickObjectGoal()
        goal.mask = self.bridge.cv2_to_imgmsg(object_info.mask_full)
        goal.reference_img = self.bridge.cv2_to_imgmsg(rgb)
        goal.depth_img = self.bridge.cv2_to_imgmsg(depth)
        goal.command = "point_at"

        rospy.loginfo("sending goal")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result.success

    def move_out_of_view(self):

        moveit.moveToNamed("camera_ready_1")
        self.is_home = True

        return True

        """
        rospy.loginfo("Using out of view service")

        print(moveit.getCurrentState())

        moveit.moveToNamed("camera_ready_1") #check if this pose is for camera to see

        self.is_home = result

        return result.success
        

        self.client.wait_for_server()

        goal = PickObjectGoal()
        goal.command = "move_out_of_view"

        rospy.loginfo("sending goal")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()

        self.is_home = result

        return result.success
        """

    def is_out_of_view(self):
        return not self.is_home


if __name__ == "__main__":
    rospy.init_node("controller", anonymous=True)
    controller = RobotController()
    controller.grasp(0.5)
