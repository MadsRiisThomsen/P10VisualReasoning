import rospy
from moveit_msgs.msg import ExecuteTrajectoryGoal
import numpy as np
np.set_printoptions(precision=3, suppress=True)
import math
import actionlib
from scipy.spatial.transform.rotation import Rotation
from bin_picking.msg import MoveRobotAction, MoveRobotGoal
from bin_picking_lib.move_robot.ur_utils import Utils


def apply_transform_real_to_moveit(pose):
    pose_local = pose.copy()
    tvec = pose_local[:3]
    orientation = Rotation.from_rotvec(pose_local[3:])
    pose_tmat = Utils.trans_and_rot_to_tmat(tvec, orientation)
    rot_z_1 = Rotation.from_euler("xyz", [0, 0, np.pi])
    rot_z_2 = Rotation.from_euler("xyz", [0, 0, np.pi / 2])
    rot_y = Rotation.from_euler("xyz", [0, -np.pi / 2, 0])
    tmat_z_1 = Utils.trans_and_rot_to_tmat([0, 0, 0], rot_z_1)
    tmat_z_2 = Utils.trans_and_rot_to_tmat([0, 0, 0], rot_z_2)
    tmat_y = Utils.trans_and_rot_to_tmat([0, 0, 0], rot_y)
    applied_tmat = tmat_z_1 @ pose_tmat @ tmat_z_2 @ tmat_y
    applied_trans, applied_rot = Utils.tmat_to_trans_and_rot(applied_tmat)
    applied_rvect = applied_rot.as_rotvec()
    return np.concatenate((applied_trans, applied_rvect))


def apply_transform_moveit_to_real(pose):
    pose_local = pose.copy()
    tvec = pose_local[:3]
    orientation = Rotation.from_rotvec(pose_local[3:])
    pose_tmat = Utils.trans_and_rot_to_tmat(tvec, orientation)
    rot_z_1 = Rotation.from_euler("xyz", [0, 0, -np.pi])
    rot_z_2 = Rotation.from_euler("xyz", [0, 0, -np.pi / 2])
    rot_y = Rotation.from_euler("xyz", [0, np.pi / 2, 0])
    tmat_z_1 = Utils.trans_and_rot_to_tmat([0, 0, 0], rot_z_1)
    tmat_z_2 = Utils.trans_and_rot_to_tmat([0, 0, 0], rot_z_2)
    tmat_y = Utils.trans_and_rot_to_tmat([0, 0, 0], rot_y)
    applied_tmat = tmat_z_1 @ pose_tmat @ tmat_y @ tmat_z_2
    applied_trans, applied_rot = Utils.tmat_to_trans_and_rot(applied_tmat)
    applied_rvect = applied_rot.as_rotvec()
    return np.concatenate((applied_trans, applied_rvect))

class MoveRobotMoveIt:
    def __init__(self, create_node=False):
        if create_node:
            rospy.init_node("moveit_test", anonymous=True)
        self.client = actionlib.SimpleActionClient("bin_picking_moveit_interface", MoveRobotAction)
        rospy.loginfo("MoveRobotMoveIt waiting for action server to come online")
        self.client.wait_for_server()
        #self.home_pose_l = [35, -300, 300, 0, 0, -0.8] # old
        self.home_pose_l = [0.035, -0.300, 0.300, 0, 0, -0.8]
        self.home_pose_gripper = [-60, -60, -110, -100, 90, -60]
        self.home_pose_suction = [-60, -60, -110, 170, -70, 100]
        self.move_out_of_view_pose = [-150, -60, -110, 170, -70, 100]
        self.default_orientation = [0, 0, 0]
        #self.gripper_tcp = [0, 0, 0.201, 0, 0, 0]
        #self.suction_tcp = [-0.193, 0, 0.08, 0, -np.pi/2, 0]

        self.camera_pose_gripper = [-60, -60, -110, -100, -90, -75]
        self.camera_pose_suction = [-5, -40, -100, -140, 0, -170]

        self.pcb_singularity_avoidance = [-70, -70, -107, -180, -147, 90]

        self.cover_closed = 20
        self.box_closed = 3

        #Part drop locations:
        self.white_cover_drop = [0.350, -0.400, 0.300, 2.89, 1.21, 0] # old
        #self.white_cover_drop = [-0.350, 0.400, 0.300, -0.61, 1.48, 0.62] #intermediate calc
        # New frames are R_z(pi) * current_transform * R_z(pi/2) * R_y(-pi/2)
        #self.white_cover_drop = [-0.350, 0.400, 0.300, 0.61, 1.48, -0.61] # this one is calibrated for new frames
        self.black_cover_drop = [200, -250, 100, 2.89, 1.21, 0] #old
        self.blue_cover_drop = [-50, -250, 100, 2.89, 1.21, 0] #old
        self.bottom_cover_drop = [-150, -350, 100, 2.89, 1.21, 0] #old
        self.pcb_drop = [-250, -450, 100, 2.89, 1.21, 0] #old

        rospy.loginfo("Move Robot interface ready")

    def movej(self, pose, acceleration=1.0, velocity=0.1, degrees=True, max_retries=3):
        pose_local = pose.copy()
        if degrees:
            for i in range(6):
                pose_local[i] = math.radians(pose_local[i])
        goal = MoveRobotGoal()
        goal.action = "joint"
        goal.joint_goal.positions = pose_local
        goal.joint_goal.velocities = np.repeat(velocity, 6)
        retry_amount = 0
        success = False
        while retry_amount < max_retries:
            self.client.send_goal(goal)
            self.client.wait_for_result()
            result = self.client.get_result()
            success = result.success
            if success:
                break
            retry_amount += 1
        return success

    def movel(self, pose, acceleration=1.0, velocity=0.2, use_mm=False, max_retries=3):
        pose_local = pose.copy()
        if use_mm:
            pose_local[0] *= 0.001
            pose_local[1] *= 0.001
            pose_local[2] *= 0.001
        pose_local = self.apply_gripper_tcp_offset(pose_local)
        pose_local = apply_transform_real_to_moveit(pose_local)
        print("moving to " + str(pose_local))
        quaternion = Rotation.from_rotvec(pose_local[3:]).as_quat()
        goal = MoveRobotGoal()
        goal.action = "cartesian"
        goal.cartesian_goal.position.x = pose_local[0]
        goal.cartesian_goal.position.y = pose_local[1]
        goal.cartesian_goal.position.z = pose_local[2]
        goal.cartesian_goal.orientation.x = quaternion[0]
        goal.cartesian_goal.orientation.y = quaternion[1]
        goal.cartesian_goal.orientation.z = quaternion[2]
        goal.cartesian_goal.orientation.w = quaternion[3]
        retry_amount = 0
        success = False
        while retry_amount < max_retries:
            self.client.send_goal(goal)
            self.client.wait_for_result()
            result = self.client.get_result()
            success = result.success
            if success:
                break
            retry_amount += 1
        return success

    def movel2(self, location, orientation, acceleration=1.0, velocity=0.2, use_mm=False, max_retries=3):
        return self.movel(np.concatenate((location, orientation)), use_mm=use_mm, max_retries=max_retries)

    def set_tcp(self, pose):
        # TODO: Implement this
        pass

    def move_to_home_suction(self, speed=1.0):
        return self.movej(self.home_pose_suction, acceleration=1.0, velocity=speed)

    def move_to_home_gripper(self, speed=1.0):
        return self.movej(self.home_pose_gripper, acceleration=1.0, velocity=speed)

    def move_to_home_l(self, speed=1.0):
        return self.movel(self.home_pose_l, acceleration=1.0, velocity=speed)

    def move_out_of_view(self, speed=2.0):
        return self.movej(self.move_out_of_view_pose, acceleration=1.0, velocity=speed)

    def open_gripper(self):
        goal = MoveRobotGoal()
        goal.action = "open_gripper"
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result.success

    def close_gripper(self, width=0, speed=5, lock=False, gripping_box=False):
        goal = MoveRobotGoal()
        goal.action = "close_gripper"
        goal.gripper_width = width
        goal.gripper_speed = speed
        goal.gripper_lock = lock
        goal.gripper_grip_box = gripping_box
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result.success

    def grasp_cover(self):
        return self.close_gripper(self.cover_closed)

    def grasp_box(self):
        return self.close_gripper(self.box_closed, gripping_box=True)

    def enable_suction(self):
        goal = MoveRobotGoal()
        goal.action = "suction_on"
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result.success

    def disable_suction(self):
        goal = MoveRobotGoal()
        goal.action = "suction_off"
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        return result.success

    def apply_gripper_tcp_offset(self, pose):
        pose_trans = pose[:3]
        pose_rot = Rotation.from_rotvec(pose[3:])
        pose_tmat = Utils.trans_and_rot_to_tmat(pose_trans, pose_rot)
        tcp_tmat = Utils.trans_and_rot_to_tmat([0, 0, -0.201], Rotation.from_rotvec([0, 0, 0]))
        new_pose_tmat = pose_tmat @ tcp_tmat
        new_pose_trans, new_pose_rot = Utils.tmat_to_trans_and_rot(new_pose_tmat)
        new_pose_rotvec = new_pose_rot.as_rotvec()
        return np.concatenate((new_pose_trans, new_pose_rotvec))


if __name__ == "__main__":
    test_pose = np.array([-0.4379,0.33657,0.33055, 0.5, 0, 0.5])
    #moveit_pose = apply_transform_real_to_moveit(test_pose)
    world_pose = apply_transform_moveit_to_real(test_pose)
    print("start pose", test_pose)
    #print("moveit_pose", moveit_pose)
    print("world_pose", world_pose)
    #mr = MoveRobotMoveIt(create_node=True)
    #mr.close_gripper(10)
    #mr.open_gripper()
    #mr.move_to_home_gripper()
    #mr.move_to_home_l()
    #mr.move_to_home_suction()
    #mr.move_out_of_view()
    #mr.movel(mr.white_cover_drop)

    #[0.350, -0.400, 0.300, 2.89, 1.21, 0] #old
    #[-0.350, 0.400, 0.300, 0.61, 1.48, -0.61] #new expected
    #a = mr.apply_gripper_tcp_offset([0.350, -0.400, 0.300, 2.89, 1.21, 0])
    #b = mr.apply_transform_real_to_moveit(a)
    #print("done")