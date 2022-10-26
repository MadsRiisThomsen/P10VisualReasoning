#!/usr/bin/env python3
import rospy
import actionlib
from bin_picking_lib.vision.surface_normal import SurfaceNormals
import cv2
import numpy as np
from bin_picking.msg import PickObjectAction, PickObjectFeedback, PickObjectResult, PickObjectGoal
from cv_bridge import CvBridge
from bin_picking_lib.move_robot.move_robot_moveit import MoveRobotMoveIt
from bin_picking_lib.aruco import Calibration
from vision_lib.ros_camera_interface import ROSCamera
from testing_resources.find_objects import FindObjects, ObjectInfo


class MoveCommand:
    def __init__(self, should_terminate_if_fails, method, *args, **kwargs):
        self.should_terminate_if_fails = should_terminate_if_fails
        self.method = method
        self.args = args
        self.kwargs = kwargs

    def __call__(self):
        success = self.method(*self.args, **self.kwargs)
        if not success and self.should_terminate_if_fails:
            rospy.logerr("Moving robot failed after retries")
            return False
        return True


class PickingNode:
    def __init__(self, testing=False):
        rospy.init_node("picking_node")
        self.action_server = actionlib.SimpleActionServer("pick_object", PickObjectAction, self.callback, auto_start=False)

        self.surface_normals = SurfaceNormals()
        self.aruco = Calibration()
        self.bridge = CvBridge()
        self.move_robot = MoveRobotMoveIt()

        if testing:
            self.camera = ROSCamera()
            self.background_img = cv2.imread("testing_resources/background_test.png")
            self.object_finder = FindObjects(self.background_img, [0, 0, 400, 400])

        self.action_server.start()
        rospy.loginfo("server started")

    def callback(self, goal: PickObjectGoal):
        rospy.loginfo("entered callback")
        feedback = PickObjectFeedback()
        feedback.status = "executing"
        self.action_server.publish_feedback(feedback)
        command = goal.command
        succeeded = True
        if command == "move_out_of_view":
            self.move_robot.move_out_of_view()
        elif command == "test":
            self.move_robot.move_out_of_view()
            img = self.camera.get_image()
            depth_img = self.camera.get_depth()
            object_to_pick = self.object_finder.find_objects(img, debug=False)[0]
            mask = object_to_pick.mask_full
            #cv2.imwrite("testing_resources/img.png", img)
            #cv2.imwrite("testing_resources/mask.bmp", mask)
            #np.save("testing_resources/depth_img.npy", depth_img)
            #cv2.imshow("a", object_to_pick.object_img_cutout_cropped)
            #cv2.waitKey()
            center, rotvec, normal_vector, relative_angle_to_z, short_vector = self.surface_normals.get_gripper_orientation(mask, depth_img, self.background_img, 0)
            self.move_robot.move_to_home_gripper(speed=3)
            self.move_robot.movel([0, -300, 300, 0, np.pi, 0], velocity=0.8, use_mm=True)
            approach_center = center + 200 * normal_vector
            pose_approach = np.concatenate((approach_center, rotvec))
            self.move_robot.movel(pose_approach, use_mm=True)
            pose_pick = np.concatenate((center - 14 * normal_vector, rotvec))
            self.move_robot.close_gripper(50)
            self.move_robot.movel(pose_pick, velocity=0.1, use_mm=True)
            gripper_close_distance = 40
            self.move_robot.close_gripper(gripper_close_distance, speed=0.5, lock=True)
            self.move_robot.movel2([center[0], center[1], 100], rotvec, use_mm=True)
            rospy.loginfo("test done")
        elif command == "pick_object":
            mask = self.bridge.imgmsg_to_cv2(goal.mask, desired_encoding="passthrough")
            reference_img = self.bridge.imgmsg_to_cv2(goal.reference_img, desired_encoding="passthrough")
            depth_img = self.bridge.imgmsg_to_cv2(goal.depth_img, desired_encoding="passthrough")
            center, rotvec, normal_vector, relative_angle_to_z, short_vector = self.surface_normals.get_gripper_orientation(mask, depth_img, reference_img, 0)
            approach_center = center + 200 * normal_vector
            pose_approach = np.concatenate((approach_center, rotvec))
            pose_pick = np.concatenate((center - 8 * normal_vector, rotvec))
            gripper_close_distance = 20
            commands = [
                MoveCommand(False, self.move_robot.move_to_home_gripper, speed=3),
                MoveCommand(True, self.move_robot.movel, [0, -300, 300, 0, np.pi, 0], velocity=0.8, use_mm=True, max_retries=3),
                MoveCommand(True, self.move_robot.movel, pose_approach, use_mm=True, max_retries=3),
                MoveCommand(True, self.move_robot.close_gripper, 50),
                MoveCommand(True, self.move_robot.movel, pose_pick, velocity=0.1, use_mm=True),
                MoveCommand(True, self.move_robot.close_gripper, gripper_close_distance, speed=0.5, lock=True),
                MoveCommand(False, rospy.sleep, 0.2),
                MoveCommand(True, self.move_robot.movel2, [center[0], center[1], 100], rotvec, use_mm=True),
                MoveCommand(True, self.move_robot.movel2, [center[0], center[1], 300], [0, np.pi, 0], use_mm=True)
            ]
            for command in commands:
                success = command()
                if not success:  # not success here means that we want to terminate if it fails
                    succeeded = False
                    break
        elif command == "place_object":
            reference_img = self.bridge.imgmsg_to_cv2(goal.reference_img, desired_encoding="passthrough")
            point = self.aruco.calibrate(reference_img, goal.place_image_x, goal.place_image_y, goal.place_world_z)
            self.move_robot.movel2(point, [0, np.pi, 0], use_mm=True)
            self.move_robot.open_gripper()
        elif command == "point_at":
            mask = self.bridge.imgmsg_to_cv2(goal.mask, desired_encoding="passthrough")
            reference_img = self.bridge.imgmsg_to_cv2(goal.reference_img, desired_encoding="passthrough")
            depth_img = self.bridge.imgmsg_to_cv2(goal.depth_img, desired_encoding="passthrough")
            center, rotvec, normal_vector, relative_angle_to_z, short_vector = self.surface_normals.get_gripper_orientation(mask, depth_img, reference_img, 0)
            self.move_robot.move_to_home_gripper(speed=3)
            position = [center[0], center[1], 300]
            self.move_robot.movel2(position, [0, np.pi, 0], use_mm=True)
            position[2] = 100
            self.move_robot.movel2(position, [0, np.pi, 0], use_mm=True)
        else:
            rospy.logerr("received invalid command" + command)
            succeeded = False

        result = PickObjectResult()
        result.success = succeeded
        self.action_server.set_succeeded(result=result)


if __name__ == "__main__":
    server = PickingNode(testing=False)