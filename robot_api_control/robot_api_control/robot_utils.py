import os
import sys
import time
import json

import rclpy
import numpy as np

from robot_api_control.constants import (
    ROBOT_ID,
    ROBOT_MODEL,
    GRIPPER_NAME,
    TOOLCHARGER_IP,
    TOOLCHARGER_PORT,
    VELOCITY,
    ACC,
    DEPTH_OFFSET,
    MIN_DEPTH,
    YAW_OFFSET,
    GRIP_OFFSET,
    BUCKET_POS,
    JSON_PATH, 
    HOME
)
from robot_api_control.transformation import Transformation
from robot_api_control.onrobot import RG
import DR_init
rclpy.init()
node = rclpy.create_node("main_controller_node", namespace=ROBOT_ID)
# Doosan API imports
try:
    from DSR_ROBOT2 import (
        movej,
        movel,
        mwait,
        get_current_posx,
        task_compliance_ctrl,
        release_force,
        check_force_condition,
        set_desired_force,
        release_compliance_ctrl,
        DR_AXIS_Z,
        DR_FC_MOD_REL,
        DR_BASE
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit(1)

# Initialize ROS and Doosan robot
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
# rclpy.init()
# node = rclpy.create_node("main_controller_node", namespace=ROBOT_ID)
DR_init.__dsr__node = node

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

class RobotUtils:
    def __init__(
        self,
        tf: Transformation,
        velocity: int = VELOCITY,
        acc: int = ACC,
        depth_offset: float = DEPTH_OFFSET,
        min_depth: float = MIN_DEPTH,
        yaw_offset: float = YAW_OFFSET,
        grip_offset: int = GRIP_OFFSET,
    ):
        """
        :param tf: Transformation helper instance
        :param velocity: joint and linear velocity (default from constants)
        :param acc: joint and linear acceleration (default from constants)
        :param depth_offset: Z-axis offset for pick depth
        :param min_depth: minimum Z for safe approach
        :param yaw_offset: yaw angle offset for gripper alignment
        :param grip_offset: value to adjust gripper width calculation
        """
        # Transformation and parameters
        self.tf = tf
        self.velocity = velocity
        self.acc = acc
        self.depth_offset = depth_offset
        self.min_depth = min_depth
        self.yaw_offset = yaw_offset
        self.grip_offset = grip_offset

        # Motion and gripper interfaces
        self.movej = movej
        self.movel = movel
        self.mwait = mwait
        self.get_current_posx = get_current_posx
        self.gripper = gripper

        # Force control interfaces
        self.task_compliance_ctrl = task_compliance_ctrl
        self.release_force = release_force
        self.check_force_condition = check_force_condition
        self.set_desired_force = set_desired_force
        self.release_compliance_ctrl = release_compliance_ctrl

        # Doosan constants
        self.DR_AXIS_Z = DR_AXIS_Z
        self.DR_FC_MOD_REL = DR_FC_MOD_REL
        self.DR_BASE = DR_BASE

    def init_robot(self, ready_pose: list = HOME):
        """
        Move robot to ready pose and open gripper.
        """
        self.movej(ready_pose, vel=self.velocity, acc=self.acc)
        self.gripper.open_gripper()
        self.mwait()

    def get_target_pos(self, camera_coords: list) -> list:
        """
        Convert camera coords [x, y, z, yaw(rad)] to robot base pose (degrees).
        """
        coords = camera_coords.copy()
        coords[-1] = np.degrees(coords[-1])
        base_pose = self.get_current_posx()[0]
        pose = self.tf.obj_pose_in_base(base_pose, coords)

        # Adjust Z for depth
        if pose[2] and sum(pose) != 0:
            pose[2] = max(pose[2] + self.depth_offset, self.min_depth)

        # Adjust yaw
        pose[-1] += self.yaw_offset
        return pose

    def get_grip_force(self, class_name: str, json_path: str = JSON_PATH) -> int:
        """
        Retrieve grip_force from JSON_PATH by class name.
        """
        if not os.path.isfile(json_path):
            raise FileNotFoundError(f"JSON file not found: {json_path}")

        with open(json_path, encoding='utf-8') as f:
            data = json.load(f)

        info = data.get(class_name)
        if not info or 'grip_force' not in info:
            raise KeyError(f"Grip force not defined for '{class_name}' in JSON")

        return int(info['grip_force'])

    def pick_and_place(
        self,
        target_pose: list,
        min_size: float,
        grip_force: int,
        bucket_pose: list = BUCKET_POS
    ):
        """
        Execute pick-and-place sequence:
          1. Move to object
          2. Grasp
          3. Lift
          4. Move to bucket
          5. Release with force control
        """
        # Approach
        self.movel(target_pose, vel=self.velocity, acc=self.acc, ref=self.DR_BASE)

        # Grasp
        width = max(int(min_size * 10) - self.grip_offset, 50)
        self.gripper.move_gripper(width_val=width, force_val=grip_force)
        while self.gripper.get_status()[0]:
            time.sleep(0.2)
        self.mwait()

        # Lift
        lift_pose = target_pose.copy()
        lift_pose[2] += 100
        self.movel(lift_pose, vel=self.velocity, acc=self.acc, ref=self.DR_BASE)

        # Move to bucket
        self.movel(bucket_pose, vel=self.velocity, acc=self.acc, ref=self.DR_BASE)

        # Release with force control
        self.task_compliance_ctrl()
        time.sleep(0.1)
        self.set_desired_force(fd=[0,0,-15,0,0,0], dir=[0,0,1,0,0,0], mod=self.DR_FC_MOD_REL)
        time.sleep(0.1)
        while not self.check_force_condition(self.DR_AXIS_Z, max=10):
            pass
        self.release_force()
        self.release_compliance_ctrl()

        # Open gripper
        self.mwait()
        self.gripper.open_gripper()
        while self.gripper.get_status()[0]:
            time.sleep(0.2)

# Example usage:
# tf = Transformation()
# utils = RobotUtils(tf)
