#!/usr/bin/env python3
"""
pick-and-place(로봇 M0609) - 음성 → 키워드 → 객체 검출 → 집기
-----------------------------------------------------------------
- /get_keyword          : std_srvs/srv/Trigger
- /obj_detect           : msg/srv/ObjectInformation
    · Request  : bool state_main
    · Response : float64[] position  # [x, y, z, yaw]
                 int64    nums
"""

import os, sys, time, json
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

# 커스텀 서비스
from msgs.srv import ObjectInformation

from robot_api_control.transformation import Transformation
# Doosan API
import DR_init
from robot_api_control.onrobot import RG

# ─── 상수 ─────────────────────────────────
ROBOT_ID, ROBOT_MODEL = "dsr01", "m0609"
VELOCITY = ACC = 100
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP, TOOLCHARGER_PORT = "192.168.1.1", "502"
DEPTH_OFFSET = -20.0
MIN_DEPTH = 2.0
YAW_OFFSET = -50
GRIP_OFFSET = 200
OBB_PKG = get_package_share_directory('OBB')
GUI_PKG = get_package_share_directory('gui')
JSON_PATH = os.path.join(GUI_PKG, 'resource', 'product_data.json')

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
        get_current_posx,
        
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


# ─── Qos ───────────────────────────
def _latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )

# ─── 메인 노드 ────────────────────────────
class RobotController(Node):
    def __init__(self):
        super().__init__("main_controller_node")
        
        # ─ 좌표 변환 유틸 준비
        self.gripper2cam_path = os.path.join(
            OBB_PKG, "resource", "T_gripper2camera.npy"
        )
        self.tf = Transformation(self.gripper2cam_path)

        # 1. 키워드 서비스
        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")
        while not self.get_keyword_client.wait_for_service(3.0):
            self.get_logger().info("⏳ Waiting for /get_keyword ...")
        self.keyword_req = Trigger.Request()

        # 2. 객체 검출 서비스
        self.obj_detect_client = self.create_client(ObjectInformation, "/obj_detect")
        while not self.obj_detect_client.wait_for_service(3.0):
            self.get_logger().info("⏳ Waiting for /obj_detect ...")
        self.obj_req = ObjectInformation.Request()
        self.obj_req.state_main = False      # 초기 상태

        # 3. Gui
        # wake-up → prompt_pub
        self.prompt_pub = self.create_publisher(Bool, "/prompt_pub", _latched_qos())
        self.prompt_sent_from_wakeup = False  # 중복 발행 방지

        # class_name publisher
        self.class_pub = self.create_publisher(
            String, "/class_name", _latched_qos()
        )
        self.init_robot()
        
    ## 로봇 구동 ##
    # -----------------------------------------------------
    # 로봇 초기 자세 + 그리퍼 open
    def init_robot(self):
        J_READY = [-9.90, 18, 27.19, -0.07,133.84 , -8.53]
        movej(J_READY, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()
    
    def get_target_pos(self, target_coords):
        """
        target_coords: 카메라 좌표계 기준의 3D 좌표 [x, y, z, yaw]
        return: 로봇 베이스 좌표계 기준의 6D 위치 (pick posx)
        """
        target_coords[-1] = np.degrees(target_coords[-1])
        robot_base = get_current_posx()[0]
        
        obj_pos = self.tf.obj_pose_in_base(robot_base, target_coords)
        
        if obj_pos[2] and sum(obj_pos) != 0:
            obj_pos[2] += DEPTH_OFFSET
            obj_pos[2] = max(obj_pos[2], MIN_DEPTH)
        
        obj_pos[-1] += YAW_OFFSET
        return obj_pos
    
    # Pick & Place
    def pick_and_place(self, posx:list, min_size:float, gripper_force:int):

        movel(posx, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        # mwait()
        
        gripper.move_gripper(
            width_val=max(int(min_size * 10)-GRIP_OFFSET, 50), 
            force_val=gripper_force)
        # ✅ 그리퍼 동작 완료 대기
        while gripper.get_status()[0]:  # busy flag = 1 → 동작 중
            time.sleep(0.2)
        mwait()  # 로봇 모션 안정화

        # 위로 올라가기 
        up_pos = posx.copy()
        up_pos[2] += 100
        movel(up_pos, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        
        # 임의 버킷 위치 예시 (x,y,z,yaw 고정)
        BUCKET_POS = [647.10, 74.75, 200, 174.23, 179.03, -157.62]
        movel(BUCKET_POS, vel=VELOCITY, acc=ACC, ref=DR_BASE)
        task_compliance_ctrl()
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR_FC_MOD_REL)
        time.sleep(0.1)
        while not check_force_condition(DR_AXIS_Z, max=10):
            pass
        release_force()
        release_compliance_ctrl()

        mwait()
        gripper.open_gripper()
        while gripper.get_status()[0]:
            time.sleep(0.2)
            
    # -----------------------------------------------------
    
    # 객체 검출 서비스 호출
    def call_obj_detect(self, enable: bool):
        self.obj_req.state_main = enable
        future = self.obj_detect_client.call_async(self.obj_req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    # -----------------------------------------------------
    
    def get_grip_force(self, class_name: str, json_path: str) -> int:
        """
        주어진 class_name에 해당하는 grip_force 값을 반환한다.
        
        Parameters:
            class_name (str): 제품 이름 (예: 'free_time')
            json_path (str): JSON 파일 경로

        Returns:
            int: grip_force 값 (없으면 None)
        """
        if not os.path.exists(json_path):
            print(f"❗ JSON 파일이 존재하지 않습니다: {json_path}")
            return None

        with open(json_path, 'r') as f:
            try:
                product_data = json.load(f)
            except json.JSONDecodeError as e:
                print(f"❌ JSON 파싱 오류: {e}")
                return None

        product_info = product_data.get(class_name)
        if not product_info:
            print(f"❌ '{class_name}' 제품 정보를 찾을 수 없습니다.")
            return None

        grip_force = product_info.get("grip_force")
        if grip_force is None:
            print(f"❗ '{class_name}'에 grip_force 항목이 없습니다.")
        return grip_force
            
    # 메인 파이프라인
    def main_pipeline(self):
        
        self.init_robot()
        # 1) 음성→키워드
        self.get_logger().info("🎤 Say 'Hello Rokey' 그리고 물체 이름을 말하세요.")
        kw_future = self.get_keyword_client.call_async(self.keyword_req)
        rclpy.spin_until_future_complete(self, kw_future)

        if not kw_future.result().success:
            self.get_logger().warn(f"키워드 인식 실패: {kw_future.result().message}")
            return

        targets = kw_future.result().message.split()
        self.get_logger().info(f"🔎 인식된 키워드: {targets}")

        # ─────────────────────────────────────────────
        # ✅ 계산 키워드 감지 시 루프 기반 반복 집기 수행
        # ─────────────────────────────────────────────
        if "계산" in targets:
            # pub Bool
            self.prompt_pub.publish(Bool(data=True))
            self.obj_req.state_main = True

            while self.obj_req.state_main:
                self.get_logger().info("🔢 '계산' 키워드 감지 — pick_and_place state=True 요청")
                resp = self.call_obj_detect(True)

                if resp.nums == 0:
                    self.get_logger().warn("💬 '계산' 객체를 찾지 못했습니다.")
                    self.call_obj_detect(False)
                    self.obj_req.state_main = False  # 루프 종료 조건
                    break

                class_name = resp.class_name
                gripper_force = self.get_grip_force(class_name, JSON_PATH)
                target_coords = list(resp.position)
                min_size = target_coords.pop()
                
                self.get_logger().info(f"📍 좌표: {resp.position} (개수 {resp.nums}) 이름: {class_name}")
                
                target_pos = self.get_target_pos(target_coords)
                self.pick_and_place(target_pos, min_size, gripper_force)
                
                # pub class_name
                # self.class_pub.publish(String(data=class_name))
                self.init_robot()
                if resp.nums > 0:
                    self.call_obj_detect(True)

            return  # 계산 작업 끝나면 함수 종료
        
# ─── main ────────────────────────────────
def main():
    tool, tcp = get_tool(), get_tcp()
    print(f"Tool: {tool}, TCP: {tcp}")
    if not tool or not tcp:
        node.get_logger().warn("❗ Tool/TCP 미설정")
        rclpy.shutdown()
        return

    node = RobotController()
    while rclpy.ok():
        node.main_pipeline()

    rclpy.shutdown()
    node.destroy_node()

if __name__ == "__main__":
    main()
