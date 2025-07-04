#!/usr/bin/env python3
"""
pick-and-place(로봇 M0609) - 음성 → 키워드 → 객체 검출 → Action Client 전송
-----------------------------------------------------------------
- /get_keyword          : std_srvs/srv/Trigger
- /obj_detect           : msg/srv/ObjectInformation
- pick_and_place action : msgs/action/PickAndPlace
"""

import os
import sys
import time
import json
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String, Int32
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# 커스텀 메시지
from msgs.srv import ObjectInformation, AdultEvent, CancelObject
from msgs.action import PickAndPlace
from robot_api_control.transformation import Transformation
from robot_api_control.constants import *

# ───── Doosan API ────────────────────────────────────────────────────
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
        get_current_posx,
    )
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()
    
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
# ─── Qos ───────────────────────────
def _latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
def latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
class RobotController(Node):
    def __init__(self):
        super().__init__('pick_and_place_controller')

        # thread control
        self.stop_event = threading.Event()
        self.work_thread = None

        # 백그라운드 executor
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self._executor.add_node(dsr_node)
        threading.Thread(target=self._executor.spin, daemon=True).start()

        # 좌표 변환 util
        self.tf = Transformation(os.path.join(OBB_PKG, 'resource', 'T_gripper2camera.npy'))

        # Service clients
        self.obj_client = self.create_client(ObjectInformation, OBJ_SERVICE)
        while not self.obj_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('⏳ Waiting for /obj_detect ...')

        self.adult_event_client = self.create_client(AdultEvent, ADULT_SERVICE)
        while not self.adult_event_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('⏳ Waiting for /adult_event ...')

        # Action client
        self.ac = ActionClient(self, PickAndPlace, PAP_ACTION)
        while not self.ac.wait_for_server(timeout_sec=3.0):
            self.get_logger().info('🚀 Waiting for pick_and_place action server ...')
        self.current_goal_handle = None

        # Topics
        self.keyword_sub = self.create_subscription(String, KEYWORD_TOPIC, self.keyword_callback, _latched_qos())
        self.gui_pub     = self.create_publisher(Int32, GUI_TOPIC, latched_qos())
        self.class_pub   = self.create_publisher(String, CLASS_TOPIC, _latched_qos())

    # 키워드 콜백
    def keyword_callback(self, msg: String):
        kw = msg.data.strip()
        self.get_logger().info(f"🗝️ Keyword: {kw}")

        if kw == "계산":
            if not (self.work_thread and self.work_thread.is_alive()):
                self.stop_event.clear()
                self.work_thread = threading.Thread(target=self.run_calculation, daemon=True)
                self.work_thread.start()
        elif kw == "정지":
            self.get_logger().warn("⚠️ Stop requested")
            self.stop_event.set()
            self.cancel_current_goal()
        elif kw in KOR2ENG_DICT:
            if not (self.work_thread and self.work_thread.is_alive()):
                self.stop_event.clear()
                eng_name = KOR2ENG_DICT[kw]
                self.work_thread = threading.Thread(target=self.go_cancel_pose, args=(eng_name,), daemon=True)
                self.work_thread.start()
        else:
            self.get_logger().warn(f"❓ 알 수 없는 키워드 '{kw}'")

    # 메인 루프
    def run_calculation(self):
        self.get_logger().info("🚀 run_calculation started")
        self.gui_pub.publish(Int32(data=1))
        adult_ok = True
        try:
            resp = self.call_obj_detect(True, "none")
            if resp and resp.adult_obj:
                print(adult_ok)
                adult_ok = self.call_adult_event(resp.adult_obj)
                self.gui_pub.publish(Int32(data=3 if adult_ok else 4))
            while not self.stop_event.is_set():
                self.get_logger().info("🌀 루프 진입")
                resp = self.call_obj_detect(True, 'none')
                if not resp or not resp.class_name:
                    time.sleep(1.0)
                    continue
                class_name = resp.class_name
                size = resp.position.pop()
                target_pose = self.get_target_pos(resp.position)
                force       = 100 if class_name in 'None Class' else self.get_grip_force(class_name)
                is_bottle = class_name in ("bacchus","terra")
                if (adult_ok and class_name == 'terra') or class_name == 'None Class':
                    goal_pose = CANCEL_POS
                else:
                    goal_pose = BUCKET_POS

                self.get_logger().info(f"📍 {class_name} 발견 → 동작 시작")
                success = self.send_goal(target_pose, goal_pose, size, is_bottle, force)
                self.get_logger().info(f"🎯 동작 완료: {success}")
                if success and goal_pose == BUCKET_POS:
                    self.class_pub.publish(String(data=class_name))
        finally:
            self.stop_event.clear()
            self.get_logger().info("✅ run_calculation finished")

    # 취소 동작
    def go_cancel_pose(self, item_name: str):
        self.get_logger().info(f"↩️ {item_name} 취소 동작 시작")
        self.cancel_current_goal()
        resp = self.call_obj_detect(True, item_name)
        if not resp:
            return
        size = resp.position.pop()
        target_pose = self.get_target_pos(resp.position)
        force = self.get_grip_force(item_name)
        success = self.send_goal(target_pose, CANCEL_POS, size, item_name in ("bacchus","terra"), force)
        self.get_logger().info("✅ 취소 위치 이동 완료" if success else "❌ 취소 실패")
        self.stop_event.clear()

    # 서비스 호출 (비동기 + polling)
    def call_obj_detect(self, enable: bool, name: str):
        req = ObjectInformation.Request()
        req.state_main = enable
        req.cancel_name = name
        fut = self.obj_client.call_async(req)
        while not fut.done() and not self.stop_event.is_set():
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    def call_adult_event(self, trigger: bool) -> bool:
        req = AdultEvent.Request()
        req.trigger = trigger
        fut = self.adult_event_client.call_async(req)
        while not fut.done() and not self.stop_event.is_set():
            time.sleep(0.05)
        return fut.result().state_adult_event if fut.done() else False

    def call_cancel_position(self, class_name: str) -> list[float]:
        req = CancelObject.Request()
        req.class_name = class_name
        fut = self.cancel_event_client.call_async(req)
        while not fut.done() and not self.stop_event.is_set():
            time.sleep(0.05)
        return list(fut.result().position) if fut.done() else []

    # 좌표/힘 계산
    def get_grip_force(self, class_name: str) -> int:
        if not os.path.exists(JSON_PATH):
            return 0
        with open(JSON_PATH) as f:
            data = json.load(f)
        return data.get(class_name, {}).get('grip_force', 0)

    def get_target_pos(self, coords):
        coords[-1] = np.degrees(coords[-1])
        base = get_current_posx()[0]
        pose = self.tf.obj_pose_in_base(base, coords)
        pose[2] = max(pose[2] + DEPTH_OFFSET, MIN_DEPTH)
        pose[-1] += YAW_OFFSET
        return pose

    # 액션 호출 (비동기 + polling)
    def send_goal(self, target_pose, goal_pose, size, is_bottle, force) -> bool:
        self.get_logger().info("🚀 send_goal 시작")
        goal = PickAndPlace.Goal(
            target_pose=target_pose,
            goal_pose=[float(x) for x in goal_pose],
            min_size=float(size),
            gripper_force=int(force),
            is_bottle=is_bottle,
        )
        gh_fut = self.ac.send_goal_async(goal, feedback_callback=self.feedback_cb)
        while not gh_fut.done() and not self.stop_event.is_set():
            time.sleep(0.05)
        if self.stop_event.is_set():
            self.cancel_current_goal()
            return False
        gh = gh_fut.result()
        if not gh.accepted:
            self.get_logger().error("❌ Goal 거부됨")
            return False
        self.current_goal_handle = gh
        res_fut = gh.get_result_async()
        while not res_fut.done() and not self.stop_event.is_set():
            time.sleep(0.05)
        if self.stop_event.is_set():
            self.cancel_current_goal()
            return False
        success = res_fut.result().result.success
        self.current_goal_handle = None
        return success

    def cancel_current_goal(self):
        if self.current_goal_handle:
            cancel_fut = self.current_goal_handle.cancel_goal_async()
            cancel_fut.add_done_callback(lambda _: self.get_logger().info("⛔ 목표 취소 요청 전송됨"))

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"🔄 Feedback: {fb.feedback}")
        if not getattr(fb, 'grip', True):
            self.get_logger().warn("⚠️ Grasp failed in feedback, canceling …")
            self.cancel_current_goal()


def main():
    node = RobotController()
    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()