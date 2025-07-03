#!/usr/bin/env python3
"""
pick-and-place(로봇 M0609) 
- VoiceNode: 음성 → 키워드(Trigger) → /voice_commands 퍼블리시
- ControllerNode: /voice_commands 구독 → 객체 검출 → Action Client 전송
"""

import os
import sys
import time
import json
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String, Int32
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# 커스텀 메시지 & 액션
from msgs.srv import ObjectInformation, AdultEvent
from msgs.action import PickAndPlace

from robot_api_control.transformation import Transformation
from robot_api_control.constants import *
# ───── Doosan API ────────────────────────────────────────────────────
# Doosan API
import DR_init
from robot_api_control.onrobot import RG

# ─── DSR Gripper ───────────────────────────
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


rclpy.init()
dsr_node = rclpy.create_node("test_node", namespace=ROBOT_ID)
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

# ─── Utility ─────────────────────────────────
def _latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )

# ─── VoiceNode ─────────────────────────────────
class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        # 1) Trigger 서비스 클라이언트 (/get_keyword)
        self.client = self.create_client(Trigger, KEYWORD_SERVICE)
        while not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('⏳ Waiting for /get_keyword service...')
        self.req = Trigger.Request()

        # 2) 퍼블리셔: keyword 결과를 String 토픽으로
        self.pub = self.create_publisher(String, 'voice_commands', 10)

        # 3) 타이머: 주기적으로 서비스 호출
        self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        fut = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.success:
            msg = String()
            msg.data = res.message
            self.pub.publish(msg)
            self.get_logger().info(f'🗣️ Published voice_commands: "{msg.data}"')


# ─── ControllerNode ───────────────────────────
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # 1) 구독: voice_commands
        self.sub = self.create_subscription(
            String, 'voice_commands', self.on_command, 10)

        # 2) 서비스/액션 클라이언트 초기화
        self.obj_client = self.create_client(ObjectInformation, OBJ_SERVICE)
        while not self.obj_client.wait_for_service(3.0):
            self.get_logger().info('⏳ Waiting for /obj_detect ...')
        self.obj_req = ObjectInformation.Request()
        self.obj_req.state_main = False

        self.adult_client = self.create_client(AdultEvent, ADULT_SERVICE)
        while not self.adult_client.wait_for_service(3.0):
            self.get_logger().info('⏳ Waiting for /adult_event ...')
        self.adult_req = AdultEvent.Request()

        self.ac = ActionClient(self, PickAndPlace, PAP_ACTION)
        while not self.ac.wait_for_server(3.0):
            self.get_logger().info('🚀 Waiting for pick_and_place action server ...')

        # 3) GUI 퍼블리셔
        self.prompt_pub = self.create_publisher(Bool, PROMPT_TOPIC, _latched_qos())
        self.class_pub  = self.create_publisher(String, CLASS_TOPIC, _latched_qos())
        self.acr_pub    = self.create_publisher(Int32,  TD_TOPIC,    _latched_qos())
        self.ab_pub     = self.create_publisher(Bool,  AB_TOPIC,    _latched_qos())

        # 4) TF, gripper
        self.tf = Transformation(os.path.join(OBB_PKG, 'resource', 'T_gripper2camera.npy'))
        # 상태 변수
        self.current_goal_handle = None
        self._cancel_flag = False

    # ── Service 호출 헬퍼
    def call_obj_detect(self, enable: bool):
        self.obj_req.state_main = enable
        fut = self.obj_client.call_async(self.obj_req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def call_adult_event(self, trigger: bool) -> bool:
        self.adult_req.trigger = trigger
        fut = self.adult_client.call_async(self.adult_req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().state_adult_event

    # ── 위치 변환 / Grip force
    def get_grip_force(self, class_name: str) -> int:
        if not os.path.exists(JSON_PATH):
            self.get_logger().error(f'JSON not found: {JSON_PATH}')
            return 0
        with open(JSON_PATH, 'r') as f:
            data = json.load(f)
        return data.get(class_name, {}).get('grip_force', 0)

    def get_target_pos(self, coords):
        coords[-1] = np.degrees(coords[-1])
        robot_base = get_current_posx()[0]
        obj = self.tf.obj_pose_in_base(robot_base, coords)
        obj[2] = max(obj[2] + DEPTH_OFFSET, MIN_DEPTH)
        obj[-1] += YAW_OFFSET
        return obj

    # ── Action send / cancel
    def send_goal(self, pose, size, force):
        goal = PickAndPlace.Goal()
        goal.target_pose   = pose
        goal.min_size      = float(size)
        goal.gripper_force = int(force)

        send_fut = self.ac.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_fut)
        gh = send_fut.result()
        if not gh.accepted:
            self.get_logger().warn('❌ Goal rejected'); return False

        self.current_goal_handle = gh
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        self.get_logger().info(f'✅ Action done: {result.message}')
        self.current_goal_handle = None
        return result.success

    def cancel_action(self):
        if self.current_goal_handle:
            self.get_logger().warn('🛑 Cancelling current goal...')
            cancel_fut = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_fut)
            self.current_goal_handle = None
        else:
            self.get_logger().info('❕ No goal to cancel.')

    def feedback_cb(self, msg):
        self.get_logger().info(f'🔄 Feedback: {msg.feedback.feedback}')

    # ── voice_commands 콜백
    def on_command(self, msg: String):
        targets = msg.data.split()
        self.get_logger().info(f'🔎 Received keywords: {targets}')

        if '정지' in targets:
            self._cancel_flag = True
            self.cancel_action()
            return

        if '계산' in targets:
            self.prompt_pub.publish(Bool(data=True))
            # 1) 성인검증
            resp = self.call_obj_detect(self.obj_req.state_main)
            if resp.adult_obj:
                ok = self.call_adult_event(resp.adult_obj)
                if not ok:
                    self.get_logger().warn('⛔ Adult denied'); 
                    self.acr_pub.publish(Int32(data=0))
                    return
                self.get_logger().info('✅ Adult approved'); 
                self.acr_pub.publish(Int32(data=1))

            # 2) 파이프라인 시작 in background
            self.obj_req.state_main = True
            self._cancel_flag = False
            threading.Thread(target=self._run_pipeline, daemon=True).start()

    def _run_pipeline(self):
        while self.obj_req.state_main and not self._cancel_flag:
            resp = self.call_obj_detect(self.obj_req.state_main)
            if resp.class_name == "":
                self.ab_pub.publish(Bool(data=True))
                break
            cls = resp.class_name
            size = resp.position.pop()
            pose = self.get_target_pos(resp.position)
            self.get_logger().info(f'📍 {cls} @ {pose}, size={size}')
            force = self.get_grip_force(cls)
            self.send_goal(pose, size, force)
            self.class_pub.publish(String(data=cls))
        self.obj_req.state_main = False
        self._cancel_flag = False


# ─── Launcher ────────────────────────────────
def main(args=None):
    #rclpy.init(args=args)
    voice = VoiceNode()
    ctrl  = ControllerNode()
    exec_ = MultiThreadedExecutor()
    exec_.add_node(voice)
    exec_.add_node(ctrl)
    try:
        exec_.spin()
    finally:
        exec_.shutdown()
        voice.destroy_node()
        ctrl.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
