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
