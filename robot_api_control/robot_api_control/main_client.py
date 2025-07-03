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
from msgs.srv import ObjectInformation, AdultEvent, CheckAdultObj
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

class RobotController(Node):
    def __init__(self):
        super().__init__('main_client_node')

        # thread control
        self.stop_event = threading.Event()
        self.work_thread = None

        # 좌표 변환 util
        self.tf = Transformation(os.path.join(OBB_PKG, 'resource', 'T_gripper2camera.npy'))

        # Object-detect service client
        self.obj_client = self.create_client(ObjectInformation, OBJ_SERVICE)
        while not self.obj_client.wait_for_service(3.0):
            self.get_logger().info('⏳ Waiting for /obj_detect ...')
        self.obj_req = ObjectInformation.Request()
        self.obj_req.state_main = False
        
        # adult-event service client
        self.adult_event_client = self.create_client(AdultEvent, ADULT_SERVICE)
        while not self.adult_event_client.wait_for_service(3.0):
            self.get_logger().info('⏳ Waiting for /adult_event ...')
        self.adult_event_req = AdultEvent.Request()

        # PickAndPlace action client
        self.ac = ActionClient(self, PickAndPlace, PAP_ACTION)
        while not self.ac.wait_for_server(3.0):
            self.get_logger().info('🚀 Waiting for pick_and_place action server ...')
        self.current_goal_handle = None

        # topics & pubs
        self.current_keyword = ""
        self.keyword_sub = self.create_subscription(
            String, KEYWORD_TOPIC, self.keyword_callback, _latched_qos())
        self.prompt_pub  = self.create_publisher(Bool, PROMPT_TOPIC, _latched_qos())
        self.class_pub   = self.create_publisher(String, CLASS_TOPIC, _latched_qos())
        self.acr_pub     = self.create_publisher(Int32, TD_TOPIC, _latched_qos())
        self.ab_pub      = self.create_publisher(Bool, AB_TOPIC, _latched_qos())

    # ─── subscription callback ────────────────────────────────
    def keyword_callback(self, msg: String):
        kw = msg.data.strip()
        self.get_logger().info(f"🗝️ Keyword: {kw}")

        if kw == "계산":
            if self.work_thread is None or not self.work_thread.is_alive():
                self.stop_event.clear()
                self.work_thread = threading.Thread(target=self.run_calculation, daemon=True)
                self.work_thread.start()
                
        elif kw == "정지":
            self.get_logger().warn("⚠️ Stop requested")
            self.stop_event.set()
            self.cancel_current_goal()
            
        elif kw in KOR2ENG_DICT:
            eng_name = KOR2ENG_DICT[kw]

            if self.work_thread is None or not self.work_thread.is_alive():
                self.stop_event.clear()
                self.work_thread = threading.Thread(
                    target=self.run_calculation,
                    args=(eng_name,),
                    daemon=True
                )
                self.work_thread.start()

        
        else:
            self.get_logger().warn(f"❓ 알 수 없는 키워드 '{kw}'")

    # ─── calculation thread ────────────────────────────────────
    def run_calculation(self):
        self.get_logger().info("🚀 run_calculation started")
        # prompt 시작
        self.prompt_pub.publish(Bool(data=True))

        # initial adult flag
        adult_verification = True

        # 첫 object detect & adult check
        resp = self.call_obj_detect(True)
        if resp is None:
            self.get_logger().warn("❗ call_obj_detect returned None — retrying in 1s")
            time.sleep(1.0)
        
        if resp.adult_obj:
            approved = self.call_adult_event(resp.adult_obj)
            self.get_logger().info('🔎 Adult check in progress')
            if not approved:
                self.get_logger().warn('⛔ Adult denied, canceling sequence')
                self.acr_pub.publish(Int32(data=0))
                adult_verification = False
            else:
                self.get_logger().info('✅ Adult approved')
                self.acr_pub.publish(Int32(data=1))
                
        self.obj_req.state_main = True
        # main pick-and-place loop
        while self.obj_req.state_main or not self.stop_event.is_set():
            resp = self.call_obj_detect(True)

            # 1) 객체가 안 보이면 대기하고 재시도
            if resp.class_name is None:
                self.get_logger().warn('🚫 No object detected → 1초 뒤 재시도')
                time.sleep(1.0)
                self.obj_req.state_main = False
                continue
            
            # 2) 객체 감지 됐으면 기존 로직 유지
            class_name  = resp.class_name
            size        = resp.position.pop()
            coords      = resp.position
            pitch       = resp.pitch
            target_pose = self.get_target_pos(coords)
            force       = 100 if class_name in 'None Class' else self.get_grip_force(class_name)
            is_bottle   = class_name in ("bacchus", "terra")
            buy_or_not  = not (not adult_verification and class_name=="terra")
            goal_pose   = BUCKET_POS if buy_or_not or class_name != 'None Class' else CANCEL_POS

            self.get_logger().info(
                f"📍 Detected: {class_name}, pose={target_pose}, size={size}, "
                f"force={force}, buy={buy_or_not}, bottle={is_bottle}, pitch={pitch}"
            )

            ok = self.send_goal(target_pose, goal_pose, size, buy_or_not, is_bottle, force, pitch)
            self.class_pub.publish(String(data=class_name))
            print(self.stop_event.is_set())
            # 3) 성공 여부와 상관없이 stop_event만 체크하고 계속 돌기
    
            # if self.stop_event.is_set():
            #     break

        self.get_logger().info("✅ run_calculation finished")
        # 워커 스레드가 종료됐음을 표시해서 다음 “계산”에도 재실행 가능
        self.work_thread = None

    # ─── service & action helpers ──────────────────────────────
    def call_obj_detect(self, enable: bool):
        self.obj_req.state_main = enable
        fut = self.obj_client.call_async(self.obj_req)
        # executor 가 이미 돌아가고 있으므로 그냥 non-blocking 으로 두고
        while rclpy.ok() and not fut.done():
            if self.stop_event.is_set():
                break            # “정지”가 들어오면 빠르게 탈출
            time.sleep(0.01)     # 10 ms sleep
        return fut.result()

    def call_adult_event(self, trigger: bool) -> bool:
        self.adult_event_req.trigger = trigger
        fut = self.adult_event_client.call_async(self.adult_event_req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().state_adult_event

    def get_grip_force(self, class_name: str) -> int:
        if not os.path.exists(JSON_PATH):
            return 0
        with open(JSON_PATH, 'r') as f:
            data = json.load(f)
        return data.get(class_name, {}).get('grip_force', 0)

    def get_target_pos(self, target_coords):
        target_coords[-1] = np.degrees(target_coords[-1])
        robot_base = get_current_posx()[0]
        obj_pos = self.tf.obj_pose_in_base(robot_base, target_coords)
        if obj_pos[2] and sum(obj_pos):
            obj_pos[2] = max(obj_pos[2] + DEPTH_OFFSET, MIN_DEPTH)
        obj_pos[-1] += YAW_OFFSET
        return obj_pos

    def send_goal(
                    self, 
                    target_pose: list, 
                    goal_pose: list, 
                    size: float, 
                    buy_or_not: bool, 
                    is_bottle: bool,
                    force: int,
                    pitch: int
                ) -> bool:

        goal = PickAndPlace.Goal()
        goal.target_pose   = target_pose
        goal.goal_pose     = [float(x) for x in goal_pose]
        goal.min_size      = float(size)
        goal.gripper_force = int(force)
        goal.buy_or_not    = buy_or_not
        goal.is_bottle     = is_bottle
        goal.pitch         = pitch
        fut = self.ac.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            return False
        self.current_goal_handle = gh
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result
        self.current_goal_handle = None
        return result.success

    def cancel_current_goal(self):
        if self.current_goal_handle and self.current_goal_handle.status == 1:
            cancel_fut = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_fut)
        self.stop_event.set()

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"🔄 Feedback: {fb.feedback}")
        if getattr(fb, 'grip', True) is False:
            self.get_logger().warn("⚠️ Grasp failed in feedback, canceling …")
            self.cancel_current_goal()


def main(args=None):
    # rclpy.init(args=args)
    node = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
