# transform_utils.py
import numpy as np
from scipy.spatial.transform import Rotation

class Transformation:
    """
    카메라->그리퍼 변환행렬(T_gripper2camera.npy)을 미리 로드해 두고,
    임의의 카메라 3D 좌표를 로봇 베이스 좌표계로 바꿔 주는 유틸리티.
    """

    def __init__(self, gripper2cam_path: str):
        # 4×4 Homogeneous transform (그리퍼 → 카메라)
        self._T_gripper2cam = np.load(gripper2cam_path)

    # ──────────────────────────────────────────────────────────
    # 내부 helper
    # ──────────────────────────────────────────────────────────
    @staticmethod
    def _make_pose_matrix(x, y, z, rx, ry, rz):
        """
        Z-Y-Z 오일러(°) → 4×4 pose 행렬
        """
        rot = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = [x, y, z]
        return T

    # ──────────────────────────────────────────────────────────
    # public API
    # ──────────────────────────────────────────────────────────
    def camera_to_base(self, camera_xyz, robot_posx):
        """
        camera_xyz: (x, y, z)  — 카메라 좌표계(mm)
        robot_posx: [x, y, z, rx, ry, rz]  — 현재 그리퍼 pose (mm, °)
        return: np.ndarray (3,)  — 베이스 좌표계(mm)
        """
        # ① homogeneous 벡터
        p_cam = np.append(np.asarray(camera_xyz, dtype=float), 1.0)

        # ② 베이스→그리퍼 행렬
        T_base_grip = self._make_pose_matrix(*robot_posx)

        # ③ 베이스→카메라 행렬 = (베이스→그리퍼) × (그리퍼→카메라)
        T_base_cam = T_base_grip @ self._T_gripper2cam

        # ④ 변환
        p_base = T_base_cam @ p_cam
        return p_base[:3]
    
    def camera_yaw_to_base(self, yaw_cam_deg: float, robot_posx):
        """
        yaw_cam_deg : 카메라 좌표계 yaw (deg)
        robot_posx  : [x, y, z, rx, ry, rz] — 현재 그리퍼의 베이스계 pose

        return: [rx, ry, rz] — 베이스 좌표계 기준의 회전 (Z-Y-Z 오일러, deg)
        """
        # 1) 베이스 → 카메라 회전 행렬
        T_bg = self._make_pose_matrix(*robot_posx)
        T_bc = T_bg @ self._T_gripper2cam
        R_bc = T_bc[:3, :3]

        # 2) 카메라계 yaw 회전 행렬 (Z축 회전)
        R_cam_yaw = Rotation.from_euler("Z", yaw_cam_deg, degrees=True).as_matrix()

        # 3) 베이스 → 객체 회전 = R_bc @ R_cam_yaw
        R_bo = R_bc @ R_cam_yaw

        # 4) 회전 행렬 → Z-Y-Z 오일러 각
        rx, ry, rz = Rotation.from_matrix(R_bo).as_euler("ZYZ", degrees=True)
        return [rx, ry, rz]

    def obj_pose_in_base(self,
                        base_pose: list,        # [x, y, z, rx, ry, rz] in base frame
                        cam_obj_xyzyaw: list    # [x, y, z, yaw_deg] in camera frame
                        ) -> list:
        """
        Compute object pose in the base frame.

        Args:
            base_pose       : [x, y, z, rx, ry, rz] of TOOL w.r.t BASE (mm, deg)
            cam_obj_xyzyaw  : [x, y, z, yaw_deg] of object in CAMERA frame

        Returns:
            [x, y, z, rx, ry, rz] in BASE frame (mm, deg)
        """
        # 1) base → tool
        T_base_tool = self._make_pose_matrix(*base_pose)

        # 2) tool → camera
        T_tool_cam = self._T_gripper2cam

        # 3) camera → object (yaw only, Z-axis rotation)
        x, y, z, yaw = cam_obj_xyzyaw
        R_cam_obj = Rotation.from_euler("Z", yaw, degrees=True).as_matrix()
        T_cam_obj = np.eye(4)
        T_cam_obj[:3, :3] = R_cam_obj
        T_cam_obj[:3, 3] = [x, y, z]

        # 4) chain all: base → tool → camera → object
        T_base_obj = T_base_tool @ T_tool_cam @ T_cam_obj

        # 5) extract position and orientation
        pos = T_base_obj[:3, 3]
        rpy = Rotation.from_matrix(T_base_obj[:3, :3]).as_euler("ZYZ", degrees=True)

        return list(pos) + list(rpy)
