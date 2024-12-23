import cv2
import numpy as np

import transforms3d as tfs
from rospy import logerr, logwarn, loginfo
from scipy.optimize import minimize

from easy_handeye.handeye_calibration import HandeyeCalibration



class HandeyeCalibrationBackendOptimize(object):
    MIN_SAMPLES = 2  # TODO: correct? this is what is stated in the paper, but sounds strange
    """Minimum samples required for a successful calibration."""

    AVAILABLE_ALGORITHMS = {
        'Li_wei'
    }

    # 非线性求解所需的定义目标函数
    def objective(x,A,B):
        q0, q1, q2, q3, t1, t2, t3 = x
        X = np.array([
            [q0**2 + q1**2 - q2**2 - q3**2, 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2), t1],
            [2*(q1*q2 + q0*q3), q0**2 - q1**2 + q2**2 - q3**2, 2*(q2*q3 - q0*q1), t2],
            [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), q0**2 - q1**2 - q2**2 + q3**2, t3],
            [0, 0, 0, 1]
        ])
        error = 0
        for i in range(int(len(A))):
            error = error +  np.linalg.norm(A[i] @ X - X @ B[i],ord='fro') **2
        return np.sum(error)

    @staticmethod
    def _msg_to_opencv(transform_msg):
        cmt = transform_msg.translation
        tr = np.array((cmt.x, cmt.y, cmt.z))
        cmq = transform_msg.rotation
        rot = tfs.quaternions.quat2mat((cmq.w, cmq.x, cmq.y, cmq.z))
        return rot, tr

    @staticmethod
    def _get_opencv_samples(samples):
        """
        Returns the sample list as a rotation matrix and a translation vector.

        :rtype: (np.array, np.array)
        """
        hand_base_rot = []
        hand_base_tr = []
        marker_camera_rot = []
        marker_camera_tr = []

        for s in samples:
            camera_marker_msg = s['optical'].transform
            (mcr, mct) = HandeyeCalibrationBackendOptimize._msg_to_opencv(camera_marker_msg)
            marker_camera_rot.append(mcr)
            marker_camera_tr.append(mct)

            base_hand_msg = s['robot'].transform
            (hbr, hbt) = HandeyeCalibrationBackendOptimize._msg_to_opencv(base_hand_msg)
            hand_base_rot.append(hbr)
            hand_base_tr.append(hbt)

        return (hand_base_rot, hand_base_tr), (marker_camera_rot, marker_camera_tr)

    def compute_calibration(self, handeye_parameters, samples, algorithm=None):
        """
        Computes the calibration through the OpenCV library and returns it.

        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """
        if algorithm is None:
            algorithm = 'Li_wei'

        loginfo('Optimize backend calibrating with algorithm {}'.format(algorithm))

        if len(samples) < HandeyeCalibrationBackendOptimize.MIN_SAMPLES:
            logwarn("{} more samples needed! Not computing the calibration".format(
                HandeyeCalibrationBackendOptimize.MIN_SAMPLES - len(samples)))
            return

        # Update data
        opencv_samples = HandeyeCalibrationBackendOptimize._get_opencv_samples(samples)
        (hand_world_rot, hand_world_tr), (marker_camera_rot, marker_camera_tr) = opencv_samples

        if len(hand_world_rot) != len(marker_camera_rot):
            logerr("Different numbers of hand-world and camera-marker samples!")
            raise AssertionError

        loginfo("Computing from %g poses..." % len(samples))

        A = []
        B = []
        N = len(hand_world_rot)
        # 计算出AB矩阵
        for i in range(int(N)-1):

            T_tool_i = np.vstack([np.hstack([hand_world_rot[i],hand_world_tr[i].reshape(-1,1)]),[np.array([0,0,0,1])]])
            T_tool_i_next = np.vstack([np.hstack([hand_world_rot[i+1],hand_world_tr[i+1].reshape(-1,1)]),[np.array([0,0,0,1])]])
            A_i = np.linalg.inv(T_tool_i) @ T_tool_i_next
            A.append(A_i)

            T_cam_i = np.vstack([np.hstack([marker_camera_rot[i],marker_camera_tr[i].reshape(-1,1)]),[np.array([0,0,0,1])]])
            T_cam_i_next = np.vstack([np.hstack([marker_camera_rot[i+1],marker_camera_tr[i+1].reshape(-1,1)]),[np.array([0,0,0,1])]])
            B_i = T_cam_i @ np.linalg.inv(T_cam_i_next)
            B.append(B_i)

        # 初始猜测
        x0 = [1, 0, 0, 0,0,0,0]
        # 定义约束
        constraints = ({
            'type': 'eq',
            'fun': lambda x: np.linalg.norm(x[:4]) - 1  # 四元数归一化约束
        }, {
            'type': 'ineq',
            'fun': lambda x: 1 - np.linalg.norm(x[4:])  # 平移向量范数约束
        }
        )
        # 定义变量的范围
        bounds = [
            (-1, 1),  # q0 = qw
            (-1, 1),  # q1 = qx
            (-1, 1),  # q2 = qy
            (-1, 1),  # q3 = qz
            (None, None),  # t1 = tx
            (None, None),  # t2 = ty
            (None, None)   # t3 = tz
        ]
        # 求解
        sol = minimize(HandeyeCalibrationBackendOptimize.objective, x0, args=(A, B), bounds=bounds,constraints=constraints)
        loginfo(f"应用非线性优化的四元数是：\n {[sol.x[1],sol.x[2],sol.x[3],sol.x[0]]}\n")
        loginfo(f"应用非线性优化的平移向量是：\n {[sol.x[4],sol.x[5],sol.x[6]]}\n")
        # print(sol.x)
        loginfo(f"误差函数最小值是：\n {sol.fun}\n")

        result_tuple = ((sol.x[4],sol.x[5],sol.x[6]), (sol.x[1],sol.x[2],sol.x[3],sol.x[0]))

        ret = HandeyeCalibration(calibration_parameters=handeye_parameters,
                                 transformation=result_tuple)

        return ret
