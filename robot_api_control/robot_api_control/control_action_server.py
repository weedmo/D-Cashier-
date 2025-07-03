#!/usr/bin/env python3
"""
ROS 2 Pick‑and‑Place Action Server for Doosan M0609
==================================================
This node exposes *all* low‑level DRL operations (move, gripper, force‑
control …) behind one action interface so that the rest of the
application can run as a *client* that merely sends goals.

Action interface (to be placed e.g. in `msgs/action/PickAndPlace.action`):
-----------------------------------------------------------------------
# Goal
float64[6] target_pose   # [ x y z rx ry rz ] in the robot‑base frame
float32      min_size     # shortest bounding‑box edge (mm)
int32        gripper_force
---
bool   success
string message
---
string feedback

Build the message and source this file in the same package as your
existing custom messages (`msgs`).
"""

import os
import sys
import time
from typing import List
import numpy as np

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node

from msgs.action import PickAndPlace
from robot_api_control.constants import *
# ───── Doosan API ────────────────────────────────────────────────────
# Doosan API
import DR_init
from robot_api_control.onrobot import RG

# ─── DSR Gripper ───────────────────────────
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("main_controller_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import (
        movej,
        movel,
        mwait,
        get_tool,
        get_tcp,
        DR_BASE,
        DR_TOOL,
        get_current_posx,
        get_tool_force,
        task_compliance_ctrl,
        
        # force control
        task_compliance_ctrl,
        release_force,
        check_force_condition,
        DR_FC_MOD_REL,
        set_desired_force,
        release_compliance_ctrl,
        DR_AXIS_Z
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()
    
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


# ───── Action Server Implementation ─────────────────────────────────
class PickAndPlaceServer(Node):
    """Wraps every DRL call behind one ROS 2 action."""

    def __init__(self):
        super().__init__("pick_and_place_server")

        # Instantiate OnRobot RG‑2 gripper
        self.gripper = RG("rg2", "192.168.1.1", 502)
        
        # Ready‑pose initialisation happens once at start‑up
        self._init_robot()

        # One action server instance
        self._action_srv = ActionServer(
            self,
            PickAndPlace,
            PAP_ACTION,
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

    # ───────────── Robot helpers ──────────────────────────────
    def _init_robot(self):
        """Home the arm and open the gripper."""
        movej(HOME, vel=VELOCITY, acc=ACC)
        self.gripper.open_gripper()
        mwait()

    def _pick_and_place(self, 
                        target_pose: List[float],
                        goal_pose: List[float],
                        min_size: float,
                        grip_force: int,
                        is_bottle:bool,
                        feedback):
        
        """Blocking pick‑and‑place routine."""
        self._init_robot()
        mwait()
        init_wrench = get_tool_force()
        init_fz     = abs(init_wrench[2])-1.0
        self.get_logger().info(f"🔎 Fz = {init_fz:.2f} N")
        # 1 Move above the object
        self.get_logger().info("▶️ Approach target …")

        ## bottle z값 예외처리
        if is_bottle:
            target_pose[2] -= 20
        movel(target_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)

        # 2 Close gripper
        width_val = max(int(min_size * 10) - GRIP_OFFSET, 50)
        self.get_logger().info(f"🤏 Closing gripper (width={width_val} , force={grip_force}) …")
        self.gripper.move_gripper(width_val=width_val, force_val=grip_force)
        while self.gripper.get_status()[0]:
            time.sleep(0.1)
        mwait()
        
        # 3 Lift up
        up_pose = target_pose.copy()
        up_pose[2] += 100.0
        movel(up_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        mwait()
        
         # ─── 🔍 잡기 실패 감지 ─────────────────────
        time.sleep(0.2)                       # 센서 안정화
        wrench = get_tool_force()             # [Fx, Fy, Fz, Tx, Ty, Tz]
        fz = abs(wrench[2])                   # 3번째 요소
        self.get_logger().info(f"🔎 Fz = {fz:.2f} N")

        if fz <= init_fz:                         # 임계값 2 N 이하 = 물체 미검출
            self.get_logger().warn("❌ Grasp failed, releasing …")
            feedback.feedback = "grasp failed"
            feedback.grip = False
            self.get_logger().info("✋ Publishing feedback: grasp failed")
            # 피드백 갱신(호출 측 스레드이므로 직접 publish X → caller가 fb 이용)
            raise RuntimeError("grasp failed")
        
        # 4 Move to bucket & place
        movel(goal_pose, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        if is_bottle:
            task_compliance_ctrl()
            time.sleep(0.1)
            set_desired_force(fd=[0, 0, -15, 0, 0, 0],
                            dir=[0, 0, 1, 0, 0, 0],
                            mod=DR_FC_MOD_REL)
            time.sleep(0.1)
            while not check_force_condition(DR_AXIS_Z, max=10):
                time.sleep(0.05)
            release_force()
            release_compliance_ctrl()
            mwait()

        self.gripper.open_gripper()
        while self.gripper.get_status()[0]:
            time.sleep(0.1)

        self._init_robot()
        self.get_logger().info("✅ Pick‑and‑place cycle finished")

    # ───────────── Action callbacks ───────────────────────────
    def _goal_cb(self, goal_request):
        self.get_logger().info("📨 Goal received")
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("✋ Cancel request received – accepting …")
        self._init_robot()
        return CancelResponse.ACCEPT

    async def _execute_cb(self, goal_handle):
        fb = PickAndPlace.Feedback()
        fb.feedback = "Starting …"
        goal_handle.publish_feedback(fb)

        try:
            # ⬇️ 실행 도중 클라이언트가 cancel을 누르면 즉시 초기화
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("❕ Cancel requested – aborting")
                self._init_robot()              # ← HOME 복귀
                goal_handle.canceled()
                result = PickAndPlace.Result()
                result.success = False
                result.message = "Canceled by client"
                return result

            # 실제 작업
            self._pick_and_place(
                list(goal_handle.request.target_pose),
                list(goal_handle.request.goal_pose),
                goal_handle.request.min_size,
                goal_handle.request.gripper_force,
                goal_handle.request.is_bottle,
                fb,
            )

            goal_handle.succeed()
            result = PickAndPlace.Result()
            result.success = True
            result.message = "Pick-and-place completed successfully"
            return result

        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")
            self._init_robot()                  # ← 예외 시에도 복귀
            goal_handle.abort()
            result = PickAndPlace.Result()
            result.success = False
            result.message = str(e)
            return result

# ───── main ─────────────────────────────────────────────────────────

def main(args=None):
    if not rclpy.ok():
        rclpy.init(args=args)
    
    server = None
    try:
        tool, tcp = get_tool(), get_tcp()
        print(f"Tool: {tool}, TCP: {tcp}")
        if not tool or not tcp:
            print("❗ Tool/TCP 미설정")
            return

        server = PickAndPlaceServer()
        server._init_robot()
        rclpy.spin(server)

    except Exception as e:
        print(f"[ERROR] {e}")
    
    finally:
        if server:
            server.destroy_node()
        rclpy.shutdown()




if __name__ == "__main__":
    main()
