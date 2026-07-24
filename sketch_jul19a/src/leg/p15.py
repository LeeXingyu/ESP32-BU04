#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光雷达人腿检测 + d/theta 实时输出
=================================

功能:
- 从激光雷达 laserscan 接口读取点云
- 通过笛卡尔聚类识别人腿候选
- 将两条腿配对成人员目标
- 输出人到雷达的距离 d 和角度 theta

坐标约定:
- +X: 雷达右侧
- +Y: 雷达前方
- theta: 0° = +X, 顺时针为正

用法:
    python p15.py
    python p15.py 192.168.1.100
    python p15.py --once
    python p15.py --interval 0.5
    python p15.py --format json
"""

import argparse
import json
import math
import time
from datetime import datetime
from typing import List, Tuple

import requests

# ROBOT_IP = "192.168.0.56"
# ROBOT_PORT = 1448
# DEFAULT_INTERVAL = 2.0
HEADERS = {"Content-Type": "application/json", "accept": "application/json"}

LASERSCAN_URL = "http://{ip}:{port}/api/core/system/v1/laserscan"

# 距离修正：测试数据比实际值大约多 36 cm
DISTANCE_OFFSET_M = -0.36

# 腿宽度范围
LEG_WIDTH_MIN = 0.03
LEG_WIDTH_MAX = 0.22

# 人员距离 / 角度过滤
PERSON_DIST_MIN = 1.0
PERSON_DIST_MAX = 3.0
PERSON_THETA_MIN = -60.0
PERSON_THETA_MAX = 60.0

# 两腿间距
LEG_PAIR_DIST_MIN = 0.10
LEG_PAIR_DIST_MAX = 0.60

# 聚类参数
CLUSTER_DIST_M = 0.10
ADAPTIVE_FACTOR = 0.005

# 通用过滤
MIN_POINTS = 3
MAX_POINTS = 60
DIST_MIN_M = 0.15
DIST_MAX_M = 10.0
CIRCULARITY_MAX = 0.70
ASPECT_MAX = 4.5


def angle_uwb(rad: float) -> float:
    """弧度转 0~360 度，按当前工程约定做 90° 偏移。"""
    deg = math.degrees(rad)
    deg = (deg + 360) % 360
    deg = (deg + 90) % 360
    return round(deg, 1)


def fetch_laserscan(ip: str) -> dict:
    """获取一次激光雷达数据。"""
    url = LASERSCAN_URL.format(ip=ip, port=ROBOT_PORT)
    try:
        resp = requests.get(url, headers=HEADERS, timeout=5)
        resp.raise_for_status()
        return resp.json()
    except Exception as exc:
        return {"error": str(exc)}


def get_valid_points(data: dict):
    """提取有效点，返回 (x_front, y_left, angle_rad) 列表。"""
    points = data.get("laser_points", [])
    result = []
    for pt in points:
        if not pt.get("valid", False):
            continue

        angle = pt.get("angle")
        distance = pt.get("distance")
        if angle is None or distance is None:
            continue

        d = float(distance) + DISTANCE_OFFSET_M
        if d <= 0 or d < DIST_MIN_M or d > DIST_MAX_M:
            continue

        a = float(angle)
        result.append((d * math.cos(a), d * math.sin(a), a))

    return result


def cluster_points(cart_pts):
    """对极坐标转平面后的点做自适应欧氏聚类。"""
    if not cart_pts:
        return []

    sorted_pts = sorted(cart_pts, key=lambda p: p[2])
    clusters = []
    current = [sorted_pts[0]]

    for i in range(1, len(sorted_pts)):
        prev = sorted_pts[i - 1]
        cur = sorted_pts[i]
        avg_d = (math.hypot(prev[0], prev[1]) + math.hypot(cur[0], cur[1])) / 2
        thresh = CLUSTER_DIST_M + ADAPTIVE_FACTOR * avg_d
        dx, dy = cur[0] - prev[0], cur[1] - prev[1]
        if math.hypot(dx, dy) <= thresh:
            current.append(cur)
        else:
            clusters.append(current)
            current = [cur]

    if current:
        clusters.append(current)

    # 首尾闭合
    if len(clusters) >= 2:
        first, last = clusters[0], clusters[-1]
        avg_d = (math.hypot(first[0][0], first[0][1]) + math.hypot(last[-1][0], last[-1][1])) / 2
        thresh = CLUSTER_DIST_M + ADAPTIVE_FACTOR * avg_d
        if math.hypot(first[0][0] - last[-1][0], first[0][1] - last[-1][1]) <= thresh:
            clusters[0] = last + first
            clusters.pop()

    return clusters


def cluster_geometry(pts) -> dict:
    """计算聚类的几何属性。"""
    n = len(pts)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    cx = sum(xs) / n
    cy = sum(ys) / n

    dists = [math.hypot(x - cx, y - cy) for x, y in zip(xs, ys)]
    avg_r = sum(dists) / n
    var = sum((d - avg_r) ** 2 for d in dists) / n
    std_r = math.sqrt(var)

    max_pw = 0.0
    for i in range(n):
        for j in range(i + 1, n):
            dd = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
            if dd > max_pw:
                max_pw = dd

    span_x = max(xs) - min(xs)
    span_y = max(ys) - min(ys)
    aspect = (max(span_x, span_y) / min(span_x, span_y)) if min(span_x, span_y) > 0.001 else 99.0
    circularity = (std_r / avg_r) if avg_r > 0.001 else 99.0
    dist_m = math.hypot(cx, cy)
    width = max_pw if max_pw > 0.005 else avg_r * 2.5

    return {
        "cx": cx,
        "cy": cy,
        "n": n,
        "width_m": width,
        "aspect": aspect,
        "circularity": circularity,
        "distance_m": dist_m,
        "max_pw_m": max_pw,
    }


def detect_legs(data: dict) -> Tuple[List[dict], List[dict]]:
    """主检测流程。"""
    cart_pts = get_valid_points(data)
    if len(cart_pts) < 6:
        return [], []

    clusters = cluster_points(cart_pts)

    legs = []
    for cl in clusters:
        geo = cluster_geometry(cl)
        if geo["n"] < MIN_POINTS or geo["n"] > MAX_POINTS:
            continue
        if geo["distance_m"] < DIST_MIN_M or geo["distance_m"] > DIST_MAX_M:
            continue
        if geo["aspect"] > ASPECT_MAX:
            continue
        if geo["circularity"] > CIRCULARITY_MAX:
            continue
        if not (LEG_WIDTH_MIN <= geo["width_m"] <= LEG_WIDTH_MAX):
            continue

        cx, cy = geo["cx"], geo["cy"]
        angle_deg = angle_uwb(math.atan2(cy, cx))

        legs.append(
            {
                "cx_m": round(cx, 4),
                "cy_m": round(cy, 4),
                "width_m": round(geo["width_m"], 4),
                "distance_m": round(geo["distance_m"], 4),
                "angle_deg": angle_deg,
                "point_count": geo["n"],
                "circularity": round(geo["circularity"], 3),
            }
        )

    people = pair_legs_cartesian(legs)
    return legs, people


def pair_legs_cartesian(legs: List[dict]) -> List[dict]:
    """按空间距离把左右腿配对成人员。"""
    if len(legs) < 2:
        return []

    used = set()
    people = []
    indexed = sorted(enumerate(legs), key=lambda x: x[1]["distance_m"])

    for k in range(len(indexed)):
        idx_i = indexed[k][0]
        if idx_i in used:
            continue

        best_j = -1
        best_d = float("inf")
        best_sep = 0.0

        for m in range(len(indexed)):
            idx_j = indexed[m][0]
            if idx_j in used or idx_j == idx_i:
                continue

            d = math.hypot(
                legs[idx_i]["cx_m"] - legs[idx_j]["cx_m"],
                legs[idx_i]["cy_m"] - legs[idx_j]["cy_m"],
            )
            if not (LEG_PAIR_DIST_MIN <= d <= LEG_PAIR_DIST_MAX):
                continue

            preference = abs(d - 0.30)
            if preference < best_d:
                best_d = preference
                best_j = idx_j
                best_sep = d

        if best_j >= 0:
            used.add(idx_i)
            used.add(best_j)

            la, lb = legs[idx_i], legs[best_j]
            mx = (la["cx_m"] + lb["cx_m"]) / 2
            my = (la["cy_m"] + lb["cy_m"]) / 2

            d = math.hypot(mx, my)
            theta_deg = angle_uwb(math.atan2(my, mx))

            people.append(
                {
                    "d_m": round(d, 4),
                    "theta_deg": theta_deg,
                    "mid_x_m": round(mx, 4),
                    "mid_y_m": round(my, 4),
                    "leg_sep_m": round(best_sep, 4),
                    "leg_left": la,
                    "leg_right": lb,
                }
            )

    return people


def filter_people(people: List[dict]) -> List[dict]:
    """按距离和角度范围过滤人员。"""
    return [
        p
        for p in people
        if PERSON_DIST_MIN <= p["d_m"] <= PERSON_DIST_MAX
        and PERSON_THETA_MIN <= p["theta_deg"] <= PERSON_THETA_MAX
    ]


def build_report(data: dict) -> dict:
    """构建统一的输出报表。"""
    if "error" in data:
        return {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "status": "error",
            "error": data["error"],
        }

    legs, people = detect_legs(data)
    people = filter_people(people)

    return {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "status": "ok",
        "source": {
            "robot_ip": ROBOT_IP,
            "robot_port": ROBOT_PORT,
            "endpoint": "laserscan",
        },
        "person_count": len(people),
        "leg_count": len(legs),
        "people": [
            {
                "id": i + 1,
                "d_m": p["d_m"],
                "theta_deg": p["theta_deg"],
                "mid_x_m": p["mid_x_m"],
                "mid_y_m": p["mid_y_m"],
                "leg_sep_cm": round(p["leg_sep_m"] * 100, 1),
            }
            for i, p in enumerate(people)
        ],
        "legs": legs,
    }


def build_follow_target(data: dict) -> dict:
    """
    给跟随逻辑使用的最小结果集。

    返回格式:
    {
        "person_count": int,
        "people": [
            {"d_m": float, "theta_deg": float}
        ]
    }
    """
    report = build_report(data)
    if report["status"] == "error":
        return {
            "status": "error",
            "error": report["error"],
            "person_count": 0,
            "people": [],
        }

    people = [
        {
            "d_m": p["d_m"],
            "theta_deg": p["theta_deg"],
        }
        for p in report["people"]
    ]

    return {
        "status": "ok",
        "person_count": len(people),
        "people": people,
    }


def get_follow_target(data: dict) -> dict | None:
    """
    返回一个最适合跟随的目标。

    如果有多个人，默认返回距离最近的那个人。
    返回值示例:
    {
        "person_count": 2,
        "target": {"d_m": 1.83, "theta_deg": 92.4},
        "people": [...]
    }
    """
    report = build_follow_target(data)
    if report["status"] == "error":
        return None

    people = sorted(report["people"], key=lambda item: item["d_m"])
    target = people[0] if people else None
    return {
        "person_count": report["person_count"],
        "target": target,
        "people": people,
    }


def fetch_follow_target(ip: str = ROBOT_IP) -> dict | None:
    """直接拉取激光雷达并返回最适合跟随的目标。"""
    return get_follow_target(fetch_laserscan(ip))


def print_result(data: dict):
    """文本格式输出。"""
    report = build_report(data)
    if report["status"] == "error":
        print(f"[{datetime.now().strftime('%H:%M:%S')}] ERROR: {report['error']}")
        return

    target = get_follow_target(data)
    if not target or not target.get("target"):
        print("person_count=0")
        return

    t = target["target"]
    print(
        f"person_count={target['person_count']} "
        f"distance={t['d_m']:.3f}m "
        f"angle={t['theta_deg']:.1f}deg"
    )


def print_json_output(data: dict):
    """JSON 格式输出。"""
    print(json.dumps(build_report(data), ensure_ascii=False))


def print_follow_output(data: dict):
    """给跟随流程使用的轻量 JSON 输出。"""
    print(json.dumps(build_follow_target(data), ensure_ascii=False))


def main():
    parser = argparse.ArgumentParser(
        description="激光雷达人腿检测 - 实时输出距离和角度",
    )
    parser.add_argument("ip", nargs="?", default=ROBOT_IP, help=f"机器人 IP (默认: {ROBOT_IP})")
    parser.add_argument("--once", "-o", action="store_true", help="只获取一次")
    parser.add_argument("--interval", "-i", type=float, default=DEFAULT_INTERVAL, help=f"刷新间隔 (默认: {DEFAULT_INTERVAL}s)")
    parser.add_argument("--json", "-j", action="store_true", help="JSON 格式输出")
    parser.add_argument("--format", "-f", choices=("text", "json", "follow"), help="输出格式")
    args = parser.parse_args()

    ip = args.ip
    output_format = args.format or ("json" if args.json else "text")

    print(f"激光雷达人腿检测 | 目标: http://{ip}:{ROBOT_PORT}/.../laserscan")
    print(f"模式: {output_format.upper()}")
    if not args.once:
        print(f"刷新间隔: {args.interval}s | 按 Ctrl+C 退出\n")

    def emit() -> None:
        data = fetch_laserscan(ip)
        if output_format == "json":
            print_json_output(data)
        elif output_format == "follow":
            print_follow_output(data)
        else:
            print_result(data)

    if args.once:
        emit()
        return

    try:
        while True:
            emit()
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n已退出。")


if __name__ == "__main__":
    main()
