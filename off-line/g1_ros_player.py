import argparse
import os
import time
from typing import List

try:
    # 本地配置（优先），避免频繁敲命令
    from . import config as cfg
except Exception:
    cfg = None

import numpy as np

import rclpy
from rclpy.node import Node

try:
    from unitree_hg.msg import LowCmd
except Exception as e:
    LowCmd = None


def load_motion(path: str) -> np.ndarray:
    arr = np.load(path)
    if not isinstance(arr, np.ndarray):
        raise TypeError("Loaded data is not a numpy.ndarray")
    if arr.ndim != 2 or arr.shape[1] < 7 + 29:
        raise ValueError(f"Unexpected array shape: {arr.shape}, expect (N, >=36)")
    return arr


def default_kp_list() -> List[float]:
    # 参考C++：前13个关节（含腿与腰 0-12）kp=100，其余kp=50
    kp = [100.0] * 13 + [50.0] * (29 - 13)
    return kp


class G1RosPlayer(Node):
    def __init__(self, data: np.ndarray, rate_hz: float, loop: bool, scale: float, clamp_pi: bool, negate: bool):
        super().__init__('g1_ros_player')
        if LowCmd is None:
            raise RuntimeError("unitree_hg.msg.LowCmd not found. Ensure ROS2 env and package are sourced.")

        self.pub = self.create_publisher(LowCmd, '/lowcmd', 10)
        self.data = data
        self.rate_hz = max(1.0, float(rate_hz))
        self.loop = loop
        self.scale = float(scale)
        self.clamp_pi = bool(clamp_pi)
        self.negate = bool(negate)
        self.kp = default_kp_list()
        self.kd = [1.0] * 29

        self.idx = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

        # 打印数据概览
        self.get_logger().info(f"motion shape={data.shape}, dtype={data.dtype}")
        head_rows = min(3, data.shape[0])
        self.get_logger().info(f"head(3) flattened:\n{data[:head_rows]}")

    def _on_timer(self):
        row = self.data[self.idx]
        joints = (row[7:7+29].astype(np.float32) * self.scale)
        if self.negate:
            joints = -joints
        if self.clamp_pi:
            # wrap to [-pi, pi]
            joints = ((joints + np.pi) % (2*np.pi)) - np.pi

        msg = LowCmd()
        # 若消息包含模式字段，则按C++设置（存在才赋值）
        if hasattr(msg, 'mode_pr'):
            msg.mode_pr = 1
        if hasattr(msg, 'mode_machine'):
            msg.mode_machine = 1  # 29DOF模式
        # 某些消息定义包含更高层字段（如 mode_pr/mode_machine），此处最小发布：仅填充 motor_cmd
        for i in range(29):
            msg.motor_cmd[i].mode = 1
            msg.motor_cmd[i].q = float(joints[i])
            msg.motor_cmd[i].dq = 0.0
            msg.motor_cmd[i].tau = 0.0
            msg.motor_cmd[i].kp = float(self.kp[i])
            msg.motor_cmd[i].kd = float(self.kd[i])

        self.pub.publish(msg)

        self.idx += 1
        if self.idx >= self.data.shape[0]:
            if self.loop:
                self.idx = 0
            else:
                self.get_logger().info("Playback finished. Shutting down.")
                rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Offline G1 29-DoF player (.npy -> /lowcmd)')
    default_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'assets', 'data',
        'Walk_1_Skeleton0_g1_29dof_rev_1_0_Tpose_new.npy'))
    parser.add_argument('--data', type=str, default=(cfg.DATA_PATH if cfg and hasattr(cfg, 'DATA_PATH') else default_path), help='Path to .npy file (shape Nx36)')
    parser.add_argument('--rate', type=float, default=(cfg.RATE_HZ if cfg and hasattr(cfg, 'RATE_HZ') else 300.0), help='Publish rate (Hz)')
    parser.add_argument('--loop', action='store_true', default=(cfg.LOOP if cfg and hasattr(cfg, 'LOOP') else True), help='Loop playback')
    parser.add_argument('--scale', type=float, default=(cfg.SCALE if cfg and hasattr(cfg, 'SCALE') else 1.0), help='Global scale factor for joint angles')
    parser.add_argument('--negate', action='store_true', default=(cfg.NEGATE if cfg and hasattr(cfg, 'NEGATE') else False), help='Negate all 29 joint angles (flip sign)')
    parser.add_argument('--no-clamp-pi', action='store_true', default=(not cfg.CLAMP_PI if cfg and hasattr(cfg, 'CLAMP_PI') else False), help='Disable [-pi, pi] wrapping')
    args = parser.parse_args()

    if LowCmd is None:
        raise RuntimeError('unitree_hg.msg.LowCmd is unavailable. Source your ROS2 workspace.')

    if not os.path.isfile(args.data):
        raise FileNotFoundError(f"File not found: {args.data}")

    data = load_motion(args.data)

    rclpy.init()
    node = G1RosPlayer(data, rate_hz=args.rate, loop=args.loop, scale=args.scale, clamp_pi=not args.no_clamp_pi, negate=args.negate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


