#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UWB 底盘跟随脚本
================

读取 BU03 标签的 Xcm/Ycm (标签相对于小车的右侧/前方距离),
结合小车在地图中的实时位姿, 将标签位置转换为地图坐标,
每 2 秒采样一个目标点, 小车按队列依次前往 (面包屑式跟随)。

坐标约定:
  - Xcm: 标签在小车右侧的距离 (cm), 正值=右方
  - Ycm: 标签在小车前方的距离 (cm), 正值=前方
  - 地图坐标单位: 米 (m)
  - yaw: 小车朝向角 (弧度), 0=地图+X方向, 逆时针为正

坐标转换:
  标签地图 X = 小车X + (Ycm·cos(yaw) - Xcm·sin(yaw)) × 0.01
  标签地图 Y = 小车Y + (Ycm·sin(yaw) + Xcm·cos(yaw)) × 0.01

工作流程:
  串口读取 Xcm/Ycm → 获取小车位姿 → 坐标转换 → 目标点入队 → MoveToAction 依次前往

使用方式:
    python uwb_follow.py                     自动检测端口
    python uwb_follow.py COM3                指定端口
    python uwb_follow.py COM3 115200         指定端口和波特率
    python uwb_follow.py --help              查看所有选项

依赖: pip install pyserial requests
"""

import json
import math
import sys
import time
import argparse
import threading
from collections import deque
from typing import Optional, Tuple

import serial
import serial.tools.list_ports
import requests


# ═══════════════════════════════════════════════════════════════════════════════
# 配  置
# ═══════════════════════════════════════════════════════════════════════════════

ROBOT_IP = "192.168.12.1"
ROBOT_PORT = 1448

# ── API 端点 ─────────────────────────────────────────────────────────────
POSE_URL       = f"http://{ROBOT_IP}:{ROBOT_PORT}/api/core/slam/v1/localization/pose"
ACTION_URL     = f"http://{ROBOT_IP}:{ROBOT_PORT}/api/core/motion/v1/actions"
ACTION_GET_URL = f"http://{ROBOT_IP}:{ROBOT_PORT}/api/core/motion/v1/actions/{{action_id}}"

HEADERS = {"Content-Type": "application/json", "accept": "application/json"}

# ── 跟随参数 ─────────────────────────────────────────────────────────────
SAMPLE_INTERVAL  = 2.0        # 目标点采样间隔 (秒)
CM_TO_M          = 0.01       # cm → m

# ── 串口 ─────────────────────────────────────────────────────────────────
SERIAL_TIMEOUT   = 0.05
SERIAL_BAUD      = 115200


# ═══════════════════════════════════════════════════════════════════════════════
# 工  具  函  数
# ═══════════════════════════════════════════════════════════════════════════════

def list_ports():
    """列出所有可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未检测到任何串口。")
        return []
    print("可用串口:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} — {p.description}")
    return ports


def auto_detect_port() -> Optional[str]:
    """自动检测 ESP32 常用串口"""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        d = p.description.lower()
        if any(k in d for k in ("cp210", "ch340", "esp32", "silicon", "serial", "uart")):
            return p.device
    return ports[0].device if ports else None


def tag_to_map(xcm: float, ycm: float,
               robot_x: float, robot_y: float, robot_yaw: float
               ) -> Tuple[float, float]:
    """
    将标签相对坐标 (cm) 转换为地图坐标 (m)。

    参数:
        xcm:       标签在小车右侧的距离 (cm)
        ycm:       标签在小车前方的距离 (cm)
        robot_x:   小车在地图中的 X 坐标 (m)
        robot_y:   小车在地图中的 Y 坐标 (m)
        robot_yaw: 小车朝向角 (弧度), 0=+X, 逆时针为正

    返回:
        (tag_x, tag_y) 标签在地图中的坐标 (m)
    """
    dx_body = ycm * CM_TO_M   # 前方 → 车身 X
    dy_body = xcm * CM_TO_M   # 右侧 → 车身 Y

    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)

    tag_x = robot_x + dx_body * cos_yaw - dy_body * sin_yaw
    tag_y = robot_y + dx_body * sin_yaw + dy_body * cos_yaw

    return tag_x, tag_y


# ═══════════════════════════════════════════════════════════════════════════════
# 机 器 人 API
# ═══════════════════════════════════════════════════════════════════════════════

class RobotAPI:
    """封装机器人 HTTP API 调用"""

    def __init__(self, ip: str = ROBOT_IP, port: int = ROBOT_PORT):
        self.base = f"http://{ip}:{port}"

    # ── 获取小车位姿 ─────────────────────────────────────────────────
    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        """
        获取小车当前位姿。
        返回 (x, y, yaw) 单位: 米, 弧度, 失败返回 None。
        """
        try:
            resp = requests.get(
                f"{self.base}/api/core/slam/v1/localization/pose",
                headers=HEADERS, timeout=5,
            )
            resp.raise_for_status()
            data = resp.json()
            x = float(data.get("x", 0.0))
            y = float(data.get("y", 0.0))
            yaw = float(data.get("yaw", 0.0))
            return x, y, yaw
        except requests.exceptions.RequestException as e:
            print(f"[API] 获取位姿失败: {e}")
            return None
        except (ValueError, TypeError, KeyError) as e:
            print(f"[API] 位姿数据解析失败: {e}")
            return None

    # ── 直接导航到地图坐标 ───────────────────────────────────────────
    def move_to(self, x: float, y: float) -> Optional[str]:
        """
        发送 MoveToAction, 小车直接前往地图坐标 (x, y)。
        返回 action_id, 失败返回 None。
        """
        action_data = {
            "action_name": "slamtec.agent.actions.MoveToAction",
            "options": {
                "target": {
                    "x": round(x, 3),
                    "y": round(y, 3),
                    "z": 0,
                },
                "move_options": {
                    "mode": 0,
                    "flags": [],
                },
            },
        }
        try:
            resp = requests.post(
                f"{self.base}/api/core/motion/v1/actions",
                headers=HEADERS, json=action_data, timeout=10,
            )
            resp.raise_for_status()
            action_id = resp.json().get("action_id")
            if action_id:
                print(f"[导航] 出发 → ({x:.3f}, {y:.3f})  "
                      f"action_id={action_id[:8]}...")
                return action_id
            else:
                print(f"[导航] 未获取到 action_id: {resp.text}")
                return None
        except requests.exceptions.RequestException as e:
            print(f"[导航] 请求失败: {e}")
            return None

    # ── 查询导航状态 ─────────────────────────────────────────────────
    def is_action_running(self, action_id: str) -> Optional[bool]:
        """
        查询导航任务是否仍在执行。
        返回 True=执行中, False=已完成, None=查询失败。
        """
        try:
            resp = requests.get(
                f"{self.base}/api/core/motion/v1/actions/{action_id}",
                headers=HEADERS, timeout=5,
            )
            resp.raise_for_status()
            data = resp.json()
            return bool(data.get("action_name"))
        except requests.exceptions.RequestException as e:
            print(f"[导航] 查询状态失败: {e}")
            return None


# ═══════════════════════════════════════════════════════════════════════════════
# 串 口 标 签 数 据 读 取 器
# ═══════════════════════════════════════════════════════════════════════════════

class UWBReader:
    """从串口持续读取 UWB 标签数据, 保留最新一帧"""

    def __init__(self, port: str, baudrate: int = SERIAL_BAUD):
        self.port = port
        self.baudrate = baudrate
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._latest_xcm: Optional[float] = None
        self._latest_ycm: Optional[float] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    @property
    def latest(self) -> Tuple[Optional[float], Optional[float]]:
        """获取最新的 Xcm, Ycm (线程安全)"""
        with self._lock:
            return self._latest_xcm, self._latest_ycm

    def start(self):
        """打开串口并启动后台读取线程"""
        self._ser = serial.Serial(self.port, self.baudrate, timeout=SERIAL_TIMEOUT)
        print(f"[串口] 已连接 {self.port} @ {self.baudrate} baud")
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """停止读取并关闭串口"""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._ser and self._ser.is_open:
            self._ser.close()
        print("[串口] 已关闭")

    def _read_loop(self):
        """后台线程: 持续读取串口, 更新最新 Xcm/Ycm"""
        buf = ""
        while self._running:
            try:
                if self._ser.in_waiting:
                    raw = self._ser.read(self._ser.in_waiting)
                    buf += raw.decode("utf-8", errors="ignore")

                    while True:
                        start = buf.find("{")
                        if start == -1:
                            break
                        depth, end = 0, -1
                        for i in range(start, len(buf)):
                            if buf[i] == "{":
                                depth += 1
                            elif buf[i] == "}":
                                depth -= 1
                                if depth == 0:
                                    end = i + 1
                                    break
                        if end == -1:
                            break
                        try:
                            obj = json.loads(buf[start:end])
                            twr = obj.get("TWR", {})
                            xcm = twr.get("Xcm")
                            ycm = twr.get("Ycm")
                            if xcm is not None and ycm is not None:
                                with self._lock:
                                    self._latest_xcm = float(xcm)
                                    self._latest_ycm = float(ycm)
                        except (json.JSONDecodeError, ValueError, TypeError):
                            pass
                        buf = buf[end:]
                else:
                    time.sleep(0.02)
            except (serial.SerialException, OSError) as e:
                if self._running:
                    print(f"[串口] 错误: {e}")
                time.sleep(0.5)


# ═══════════════════════════════════════════════════════════════════════════════
# 跟  随  控  制  器
# ═══════════════════════════════════════════════════════════════════════════════

class FollowController:
    """
    面包屑式跟随控制器。

    工作方式:
      1. 每 SAMPLE_INTERVAL 秒从 UWB 读取标签位置
      2. 结合小车位姿计算标签在地图中的坐标
      3. 目标点入队
      4. 小车空闲时自动 MoveToAction 前往队首坐标
      5. 到达后弹出, 继续下一个
    """

    def __init__(self, port: str, baudrate: int = SERIAL_BAUD):
        self.api = RobotAPI()
        self.uwb = UWBReader(port, baudrate)

        # 目标点队列: (x, y) 地图坐标 (米)
        self._queue: deque = deque()
        self._queue_lock = threading.Lock()

        # 导航状态
        self._current_action_id: Optional[str] = None
        self._current_target: Optional[Tuple[float, float]] = None

        # 计数器
        self._total_sampled = 0
        self._total_reached = 0

        self._running = False

    # ── 队列操作 ──────────────────────────────────────────────────────
    def _queue_push(self, x: float, y: float):
        with self._queue_lock:
            self._queue.append((x, y))
        print(f"[队列] 目标点入队: ({x:.3f}, {y:.3f})  "
              f"队列长度: {len(self._queue)}")

    def _queue_pop(self) -> Optional[Tuple[float, float]]:
        with self._queue_lock:
            if self._queue:
                item = self._queue.popleft()
                print(f"[队列] 目标点出队: ({item[0]:.3f}, {item[1]:.3f})  "
                      f"剩余: {len(self._queue)}")
                return item
        return None

    def _queue_peek(self) -> Optional[Tuple[float, float]]:
        with self._queue_lock:
            return self._queue[0] if self._queue else None

    @property
    def queue_size(self) -> int:
        with self._queue_lock:
            return len(self._queue)

    # ── 主循环 ────────────────────────────────────────────────────────
    def run(self):
        """主控制循环"""
        self._running = True
        self.uwb.start()

        print(f"\n{'='*55}")
        print(f"  UWB 底盘跟随 — 面包屑模式 (直接坐标)")
        print(f"  机器人 IP: {ROBOT_IP}:{ROBOT_PORT}")
        print(f"  采样间隔: {SAMPLE_INTERVAL} 秒")
        print(f"  导航方式: MoveToAction (直接坐标)")
        print(f"  Ctrl+C 退出")
        print(f"{'='*55}\n")

        last_sample_time = 0.0

        try:
            while self._running:
                now = time.time()

                # ── 每隔 SAMPLE_INTERVAL 秒采样一次 ──
                if now - last_sample_time >= SAMPLE_INTERVAL:
                    self._sample()
                    last_sample_time = now

                # ── 检查当前导航是否完成 ──
                self._check_navigation()

                # ── 如果空闲且队列非空, 前往下一个目标点 ──
                if self._current_action_id is None and self.queue_size > 0:
                    self._go_to_next()

                time.sleep(0.2)

        except KeyboardInterrupt:
            print("\n用户中断。")
        finally:
            self._running = False
            self.uwb.stop()
            self._print_summary()

    def _sample(self):
        """采样一次: 读取 UWB → 获取位姿 → 坐标转换 → 目标点入队"""
        xcm, ycm = self.uwb.latest
        if xcm is None or ycm is None:
            print("[采样] UWB 数据不可用, 跳过")
            return

        pose = self.api.get_pose()
        if pose is None:
            print("[采样] 无法获取小车位姿, 跳过")
            return
        robot_x, robot_y, robot_yaw = pose

        # 坐标转换
        tag_x, tag_y = tag_to_map(xcm, ycm, robot_x, robot_y, robot_yaw)

        self._total_sampled += 1
        print(f"[采样] #{self._total_sampled}  "
              f"UWB=({xcm:.0f}, {ycm:.0f})cm  "
              f"小车=({robot_x:.3f}, {robot_y:.3f}, yaw={math.degrees(robot_yaw):.1f}°)  "
              f"→ 地图=({tag_x:.3f}, {tag_y:.3f})")

        self._queue_push(tag_x, tag_y)

    def _go_to_next(self):
        """从队列取下一个目标点并发起导航"""
        target = self._queue_peek()
        if target is None:
            return
        x, y = target

        action_id = self.api.move_to(x, y)
        if action_id:
            self._current_action_id = action_id
            self._current_target = (x, y)
        else:
            print(f"[导航] 前往 ({x:.3f}, {y:.3f}) 失败, 跳过")
            self._queue_pop()
            time.sleep(1.0)

    def _check_navigation(self):
        """检查当前导航任务是否完成"""
        if self._current_action_id is None:
            return

        running = self.api.is_action_running(self._current_action_id)
        if running is None:
            return  # 查询失败, 下轮再试

        if not running:
            tx, ty = self._current_target or (0, 0)
            print(f"[导航] 已到达 → ({tx:.3f}, {ty:.3f})")
            self._total_reached += 1
            self._queue_pop()
            self._current_action_id = None
            self._current_target = None

    def _print_summary(self):
        print(f"\n{'='*55}")
        print(f"  跟随结束")
        print(f"  共采样目标点: {self._total_sampled}")
        print(f"  已到达:       {self._total_reached}")
        print(f"  队列剩余:     {self.queue_size}")
        print(f"{'='*55}")


# ═══════════════════════════════════════════════════════════════════════════════
# 入  口
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="UWB 底盘跟随 — 面包屑式标签追踪 (直接坐标导航)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python uwb_follow.py                       自动检测端口
  python uwb_follow.py COM3                  指定端口
  python uwb_follow.py COM3 115200           指定端口和波特率
  python uwb_follow.py --interval 1.5        采样间隔 1.5 秒
  python uwb_follow.py --ip 192.168.12.1     指定机器人 IP
        """,
    )
    parser.add_argument("port", nargs="?", default=None,
                        help="串口端口, 不指定则自动检测")
    parser.add_argument("baudrate", nargs="?", type=int, default=SERIAL_BAUD,
                        help=f"波特率 (默认: {SERIAL_BAUD})")
    parser.add_argument("--interval", "-i", type=float, default=SAMPLE_INTERVAL,
                        help=f"目标点采样间隔 (秒) (默认: {SAMPLE_INTERVAL})")
    parser.add_argument("--ip", type=str, default=ROBOT_IP,
                        help=f"机器人 IP (默认: {ROBOT_IP})")
    args = parser.parse_args()

    # 更新全局配置
    global ROBOT_IP, SAMPLE_INTERVAL
    ROBOT_IP = args.ip
    SAMPLE_INTERVAL = args.interval

    # 串口
    port = args.port or auto_detect_port()
    if port is None:
        print("错误: 未找到可用串口。请手动指定端口。")
        list_ports()
        sys.exit(1)

    controller = FollowController(port, args.baudrate)
    controller.api = RobotAPI(ip=ROBOT_IP)
    controller.run()


if __name__ == "__main__":
    main()
