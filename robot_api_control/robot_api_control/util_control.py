#!/usr/bin/env python3
import os, time, json
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String

from msgs.srv import ObjectInformation
from robot_api_control.transformation import Transformation
from robot_api_control.constants import  BUCKET_POS       # 예시
from robot_api_control.robot_utils import RobotUtils                                 # 방금 만든 파일

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')

        # ── RobotUtils 준비 ────────────────────────────────
        tf = Transformation(os.path.join(
            os.getenv('OBB_PKG', '/opt/ros/obb_pkg'),
            'resource',
            'T_gripper2camera.npy'
        ))
        self.utils = RobotUtils(tf)

        # ── 객체 검출 서비스 클라이언트 ───────────────────
        self.obj_cli = self.create_client(ObjectInformation, '/obj_detect')
        while not self.obj_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('⏳ Waiting for /obj_detect ...')

        # ── 최초 로봇 초기 자세 ────────────────────────────
        self.utils.init_robot()

        # ── 주기적 타이머: 0.5 s마다 pick 시도 ─────────────
        self.create_timer(0.5, self.timer_cb)

    # ────────────────────────────────────────────────────
    def timer_cb(self):
        # 1) 객체 검출 요청
        req = ObjectInformation.Request()
        req.state_main = True
        fut = self.obj_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()

        # 2) 검출 실패 시 패스
        if res.class_name == "":
            self.get_logger().debug('No object.')
            return

        # 3) 좌표/크기/그립 force 계산
        class_name = res.class_name
        min_size   = res.position.pop()     # [x,y,z,yaw, size] → size 분리
        pose_in_base = self.utils.get_target_pos(res.position)
        grip_force = self.utils.get_grip_force(class_name)

        self.get_logger().info(f'📍 {class_name} → pick @ {pose_in_base}')

        # 4) Pick-and-Place 실행
        try:
            self.utils.pick_and_place(
                target_pose = pose_in_base,
                min_size    = min_size,
                grip_force  = grip_force,
                bucket_pose = BUCKET_POS
            )
        except Exception as e:
            self.get_logger().error(f'PickAndPlace failed: {e}')

# ── main ────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
