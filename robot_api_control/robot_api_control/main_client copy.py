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
import traceback
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32
from action_msgs.msg import GoalStatus
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# 커스텀 메시지
from msgs.srv import ObjectInformation, AdultEvent
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
        self.create_timer(1.0, self._check_thread)
        
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
        self.gui_pub     = self.create_publisher(Int32, GUI_TOPIC, _latched_qos())
        self.class_pub   = self.create_publisher(String, CLASS_TOPIC, _latched_qos())


    def _check_thread(self):
        if self.work_thread and not self.work_thread.is_alive():
            self.get_logger().info("🎉 run_calculation thread finished")
            # 1) 스레드가 끝났으면 join 으로 완전 정리             self.work_thread.join()
           # 2) 레퍼런스 날려 버리기
            self.work_thread = None
        
    # ─── subscription callback ────────────────────────────────
    def keyword_callback(self, msg: String):
        kw = msg.data.strip()
        self.get_logger().info(f"🗝️ Keyword: {kw}")

        if kw == "계산":
            if self.work_thread is None or not self.work_thread.is_alive():
                self.stop_event.clear()
                self.obj_req.state_main = True
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
                                target=self.go_cancel_pose,
                                args=(eng_name,),  # ← 여기서 eng_name을 전달
                                daemon=True
                                )
                self.work_thread.start()

        else:
            self.get_logger().warn(f"❓ 알 수 없는 키워드 '{kw}'")

    # ─── calculation thread ────────────────────────────────────
    def run_calculation(self):
        self.get_logger().info("🚀 run_calculation started")
        self.gui_pub.publish(Int32(data=1))  # UI: 시작

        adult_verification = True  # 초기 설정

        try:
            # ─── 1. 첫 객체 탐지 및 성인인증 ─────────────────────
            resp = self.call_obj_detect(True, "none")
            if resp is None or resp.class_name in ("", None):
                self.get_logger().warn('🚫 No object detected → 1초 뒤 재시도')
                time.sleep(1.0)
                return

            if resp.adult_obj:
                approved = self.call_adult_event(resp.adult_obj)
                self.get_logger().info('🔎 Adult check in progress')
                if not approved:
                    self.get_logger().warn('⛔ Adult denied, canceling sequence')
                    adult_verification = False
                    self.gui_pub.publish(Int32(data=4))  # UI: 거절
                else:
                    self.get_logger().info('✅ Adult approved')
                    self.gui_pub.publish(Int32(data=3))  # UI: 승인

            last_log = time.time()
            # ─── 2. 메인 Pick-and-Place 루프 ─────────────────────
            while not self.stop_event.is_set():
                now = time.time()
                if now - last_log > 5.0:
                    self.get_logger().info("🧭 5초 경과: 여전히 루프 안에 있음")
                    last_log = now
                self.get_logger().info("🌀 루프 진입")
                resp = self.call_obj_detect(True, "none")
                self.get_logger().info("📡 call_obj_detect 완료")
                # 1) 객체가 감지되지 않음
                if resp is None or resp.class_name in ("", None):
                    self.get_logger().warn('🚫 No object detected → 1초 뒤 재시도')
                    time.sleep(1.0)
                    continue

                # 2) 객체 감지됨
                class_name  = resp.class_name
                size        = resp.position.pop()
                coords      = resp.position
                target_pose = self.get_target_pos(coords)
                force       = 100 if class_name in 'None Class' else self.get_grip_force(class_name)
                is_bottle   = class_name in ("bacchus", "terra")
                goal_pose   = BUCKET_POS if  class_name != 'None Class' and adult_verification else CANCEL_POS

                adult_verification = True # 재성 추가
                
                self.get_logger().info(
                    f"📍 Detected: {class_name}, pose={target_pose}, size={size}, goal={goal_pose} "
                    f"force={force} bottle={is_bottle} adult={adult_verification}"
                )

                # 3) 실제 동작 실행
                ok = self.send_goal(target_pose, goal_pose, size, is_bottle, force)
                self.get_logger().info(f"🎯 목표 동작 완료: 성공 여부 = {ok}")
                
                # 4) 완료 후 class_name 퍼블리시 (버킷으로 간 경우만)
                if ok and np.array_equal(goal_pose, BUCKET_POS):
                    self.class_pub.publish(String(data=class_name))
                    self.get_logger().info(f"📤 class_name 퍼블리시됨: {class_name}")

        finally:
            # ─── 3. 루프 종료 후 정리 ─────────────────────
            self.stop_event.clear()
            import threading
            self.get_logger().info(f"🧵 run_calculation 내부에서 현재 스레드: {threading.current_thread().name}")
            self.get_logger().info("🧵 run_calculation 끝 직전")

        # 그리고 마지막에 꼭 명시적 return 추가!
        return

    def go_cancel_pose(self, item_name: str):
        """
        로봇을 CANCEL_POS 로 이동시키는 전용 쓰레드.
        item_name 은 로그 정도로만 사용.
        """
        self.get_logger().info(f"↩️  {item_name} 취소 동작 시작")

        self.cancel_current_goal()
        lists = self.call_obj_detect(True, item_name).position
        size        = lists.pop()
        target_pose = self.get_target_pos(lists)
        force       = 100 if item_name in 'None Class' else self.get_grip_force(item_name)
        is_bottle   = item_name in ("bacchus", "terra")
        
        # CANCEL_POS 로 이동 지령
        ok = self.send_goal(
            target_pose=target_pose,   
            goal_pose=CANCEL_POS,
            size=size,
            is_bottle=is_bottle,
            force=force
        )

        if ok:
            self.get_logger().info("✅ 로봇이 CANCEL_POS 로 이동 완료")
            self.class_pub.publish(String(data="CANCEL_DONE"))
        else:
            self.get_logger().error("❌ CANCEL_POS 이동 실패")

        # self.stop_event.clear()

    # ─── service & action helpers ──────────────────────────────
    def call_obj_detect(self, enable: bool=True, name:String="none"):
        # 1) 매번 새 Request 생성
        req = ObjectInformation.Request()
        req.state_main = enable
        req.cancel_name = name
        # 2) 비동기 호출
        fut = self.obj_client.call_async(req)
        self.get_logger().info("📞 call_obj_detect 비동기 호출")

        # 3) spin_until_future_complete로 깔끔하게 대기
        rclpy.spin_until_future_complete(self, fut)
        self.get_logger().info("📬 call_obj_detect 응답 수신")

        return fut.result()

    def call_adult_event(self, trigger: bool) -> bool:
        self.adult_event_req.trigger = trigger
        fut = self.adult_event_client.call_async(self.adult_event_req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result().state_adult_event
    
    ######################################################

    ## griper 좌표 계산
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
    
    ################################################33
    ## action 
    def send_goal(
                    self, 
                    target_pose: list, 
                    goal_pose: list, 
                    size: float, 
                    is_bottle: bool,
                    force: int,
                ) -> bool:
        self.get_logger().info("🚀 send_goal 시작")
        goal               = PickAndPlace.Goal()
        goal.target_pose   = target_pose
        goal.goal_pose     = [float(x) for x in goal_pose]
        goal.min_size      = float(size)
        goal.gripper_force = int(force)
        goal.is_bottle     = is_bottle
        fut = self.ac.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error("❌ Goal 거부됨")
            return False
        self.current_goal_handle = gh
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        self.get_logger().info("✅ 결과 수신 완료")
        result = res_fut.result().result
        self.current_goal_handle = None
        return result.success

    def cancel_current_goal(self):
        self.get_logger().info("start cancel_goal")
        if self.current_goal_handle and self.current_goal_handle.status == 1:
            self.get_logger().info("cancel_goal ~ing")
            cancel_fut = self.current_goal_handle.cancel_goal_async()
            self.get_logger().info("complete_ca ncel_goal")
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
