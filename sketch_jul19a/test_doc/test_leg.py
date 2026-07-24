#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import math
import time
import traceback
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import requests


ROBOT_IP = "192.168.0.56"
ROBOT_PORT = 1448
LASERSCAN_URL = "http://{ip}:1448/api/core/system/v1/laserscan"
POSE_URL = "http://{ip}:1448/api/core/slam/v1/localization/pose"
ACTION_URL = "http://{ip}:1448/api/core/motion/v1/actions"
HEADERS = {"Content-Type": "application/json", "accept": "application/json"}
DEFAULT_INTERVAL = 2.0
SESSION = requests.Session()
SESSION.trust_env = False

# Detection tuning.
DISTANCE_OFFSET_M = -0.36
LEG_WIDTH_MIN = 0.03
LEG_WIDTH_MAX = 0.22
PERSON_DIST_MIN = 1.0
PERSON_DIST_MAX = 3.0
PERSON_THETA_MIN = 45.0
PERSON_THETA_MAX = 135.0
LEG_PAIR_DIST_MIN = 0.10
LEG_PAIR_DIST_MAX = 0.60
CLUSTER_DIST_M = 0.10
ADAPTIVE_FACTOR = 0.005
MIN_POINTS = 3
MAX_POINTS = 60
DIST_MIN_M = 0.15
DIST_MAX_M = 10.0
CIRCULARITY_MAX = 0.70
ASPECT_MAX = 4.5


def format_time(dt: Optional[datetime]) -> str:
    if dt is None:
        return "N/A"
    return dt.strftime("%H:%M:%S.%f")[:-3]


def format_duration_ms(start: Optional[datetime], end: Optional[datetime]) -> str:
    if start is None or end is None:
        return "N/A"
    return f"{(end - start).total_seconds() * 1000:.2f}ms"


def angle_360(rad: float) -> float:
    deg = math.degrees(rad)
    deg = (deg + 360.0) % 360.0
    deg = (deg + 90.0) % 360.0
    return round(deg, 1)


def fetch_laserscan(ip: str) -> Tuple[dict, Optional[datetime], Optional[datetime]]:
    url = LASERSCAN_URL.format(ip=ip)
    t_start = datetime.now()
    try:
        resp = SESSION.get(url, headers=HEADERS, timeout=5)
        resp.raise_for_status()
        t_end = datetime.now()
        return resp.json(), t_start, t_end
    except Exception as exc:
        t_end = datetime.now()
        return {"error": str(exc)}, t_start, t_end


def get_valid_points(data: dict) -> List[Tuple[float, float, float]]:
    points = data.get("laser_points", [])
    result: List[Tuple[float, float, float]] = []
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


def cluster_points(cart_pts: List[Tuple[float, float, float]]) -> List[List[Tuple[float, float, float]]]:
    if not cart_pts:
        return []

    sorted_pts = sorted(cart_pts, key=lambda p: p[2])
    clusters: List[List[Tuple[float, float, float]]] = []
    current = [sorted_pts[0]]

    for i in range(1, len(sorted_pts)):
        prev = sorted_pts[i - 1]
        cur = sorted_pts[i]
        avg_d = (math.hypot(prev[0], prev[1]) + math.hypot(cur[0], cur[1])) / 2.0
        thresh = CLUSTER_DIST_M + ADAPTIVE_FACTOR * avg_d
        if math.hypot(cur[0] - prev[0], cur[1] - prev[1]) <= thresh:
            current.append(cur)
        else:
            clusters.append(current)
            current = [cur]

    if current:
        clusters.append(current)

    if len(clusters) >= 2:
        first, last = clusters[0], clusters[-1]
        avg_d = (math.hypot(first[0][0], first[0][1]) + math.hypot(last[-1][0], last[-1][1])) / 2.0
        thresh = CLUSTER_DIST_M + ADAPTIVE_FACTOR * avg_d
        if math.hypot(first[0][0] - last[-1][0], first[0][1] - last[-1][1]) <= thresh:
            clusters[0] = last + first
            clusters.pop()

    return clusters


def cluster_geometry(pts: List[Tuple[float, float, float]]) -> Dict[str, float]:
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
    min_span = min(span_x, span_y)
    aspect = (max(span_x, span_y) / min_span) if min_span > 0.001 else 99.0
    circularity = (std_r / avg_r) if avg_r > 0.001 else 99.0
    dist_m = math.hypot(cx, cy)
    width = max_pw if max_pw > 0.005 else avg_r * 2.5

    return {
        "cx": cx,
        "cy": cy,
        "n": float(n),
        "width_m": width,
        "aspect": aspect,
        "circularity": circularity,
        "distance_m": dist_m,
        "max_pw_m": max_pw,
    }


def pair_legs_cartesian(legs: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    if len(legs) < 2:
        return []

    used = set()
    people = []
    indexed = sorted(enumerate(legs), key=lambda x: x[1]["distance_m"])

    for idx_i, _ in indexed:
        if idx_i in used:
            continue

        best_j = -1
        best_pref = float("inf")
        best_sep = 0.0

        for idx_j, _ in indexed:
            if idx_j in used or idx_j == idx_i:
                continue
            d = math.hypot(legs[idx_i]["cx_m"] - legs[idx_j]["cx_m"],
                           legs[idx_i]["cy_m"] - legs[idx_j]["cy_m"])
            if not (LEG_PAIR_DIST_MIN <= d <= LEG_PAIR_DIST_MAX):
                continue
            pref = abs(d - 0.30)
            if pref < best_pref:
                best_pref = pref
                best_j = idx_j
                best_sep = d

        if best_j >= 0:
            used.add(idx_i)
            used.add(best_j)
            la = legs[idx_i]
            lb = legs[best_j]
            mx = (la["cx_m"] + lb["cx_m"]) / 2.0
            my = (la["cy_m"] + lb["cy_m"]) / 2.0
            d = math.hypot(mx, my)
            theta_deg = angle_360(math.atan2(my, mx))
            people.append({
                "d_m": round(d, 4),
                "theta_deg": theta_deg,
                "mid_x_m": round(mx, 4),
                "mid_y_m": round(my, 4),
                "leg_sep_m": round(best_sep, 4),
                "leg_left": la,
                "leg_right": lb,
            })

    return people


def detect_legs(data: dict) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    cart_pts = get_valid_points(data)
    if len(cart_pts) < 6:
        return [], []

    legs: List[Dict[str, Any]] = []
    for cluster in cluster_points(cart_pts):
        geo = cluster_geometry(cluster)
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

        legs.append({
            "cx_m": round(geo["cx"], 4),
            "cy_m": round(geo["cy"], 4),
            "width_m": round(geo["width_m"], 4),
            "distance_m": round(geo["distance_m"], 4),
            "angle_deg": angle_360(math.atan2(geo["cy"], geo["cx"])),
            "point_count": int(geo["n"]),
            "circularity": round(geo["circularity"], 3),
        })

    return legs, pair_legs_cartesian(legs)


def filter_people(people: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [
        p for p in people
        if PERSON_DIST_MIN <= p["d_m"] <= PERSON_DIST_MAX
        and PERSON_THETA_MIN <= p["theta_deg"] <= PERSON_THETA_MAX
    ]


def print_result(data: dict, timing: dict) -> None:
    if "error" in data:
        print(f"[{format_time(datetime.now())}] ERROR: {data['error']}")
        return

    legs, people = detect_legs(data)
    people = filter_people(people)

    t_req_start = timing.get("t_request_start")
    t_req_end = timing.get("t_request_end")
    t_ana_start = timing.get("t_analysis_start")
    t_ana_end = timing.get("t_analysis_end")

    req_duration = format_duration_ms(t_req_start, t_req_end)
    ana_duration = format_duration_ms(t_ana_start, t_ana_end)
    total_duration = format_duration_ms(t_req_start, t_ana_end)

    print()
    print("=" * 70)
    print("[time]")
    print("-" * 62)
    print(f"request start:    {format_time(t_req_start)}")
    print(f"request end:      {format_time(t_req_end)}")
    print(f"analysis start:   {format_time(t_ana_start)}")
    print(f"analysis end:     {format_time(t_ana_end)}")
    print("-" * 62)
    print(f"fetch elapsed:    {req_duration}")
    print(f"analysis elapsed: {ana_duration}")
    print(f"total elapsed:    {total_duration}")
    print("-" * 62)
    print()
    print("[result]")

    if not people:
        print(">>> result: no person")
        if legs:
            print(f"    detected {len(legs)} leg candidates but no pair")
            for i, leg in enumerate(legs, start=1):
                print(
                    f"    leg{i}: distance={leg['distance_m']:.2f}m, "
                    f"angle={leg['angle_deg']:.1f}deg, width={leg['width_m'] * 100:.0f}cm"
                )
    else:
        print(f">>> result: detected {len(people)} person(s)")
        for i, p in enumerate(people, start=1):
            print("  " + "-" * 62)
            print(f"  person {i}")
            print(f"  d (distance) = {p['d_m']:.3f} m")
            print(f"  theta        = {p['theta_deg']:.1f} deg")
            print(f"  mid(x,y)     = ({p['mid_x_m']:+.3f}, {p['mid_y_m']:+.3f}) m")
            print(f"  leg_sep      = {p['leg_sep_m'] * 100:.1f} cm")
    print()
    print("=" * 70)


def print_json_output(data: dict, timing: dict) -> None:
    if "error" in data:
        print(json.dumps({"error": data["error"]}, ensure_ascii=False))
        return

    legs, people = detect_legs(data)
    people = filter_people(people)

    def safe_ms(start: Optional[datetime], end: Optional[datetime]) -> Optional[float]:
        if start is None or end is None:
            return None
        return round((end - start).total_seconds() * 1000.0, 2)

    output = {
        "timestamp": datetime.now().isoformat(),
        "timing": {
            "request_start": format_time(timing.get("t_request_start")),
            "request_end": format_time(timing.get("t_request_end")),
            "analysis_start": format_time(timing.get("t_analysis_start")),
            "analysis_end": format_time(timing.get("t_analysis_end")),
            "request_duration_ms": safe_ms(timing.get("t_request_start"), timing.get("t_request_end")),
            "analysis_duration_ms": safe_ms(timing.get("t_analysis_start"), timing.get("t_analysis_end")),
            "total_duration_ms": safe_ms(timing.get("t_request_start"), timing.get("t_analysis_end")),
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
                "leg_sep_cm": round(p["leg_sep_m"] * 100.0, 1),
            }
            for i, p in enumerate(people)
        ],
    }
    print(json.dumps(output, ensure_ascii=False, indent=2))


class RobotAPI:
    def __init__(self, ip: str = ROBOT_IP, port: int = ROBOT_PORT):
        self.base = f"http://{ip}:{port}"

    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            resp = SESSION.get(f"{self.base}/api/core/slam/v1/localization/pose", headers=HEADERS, timeout=5)
            resp.raise_for_status()
            data = resp.json()
            return float(data.get("x", 0.0)), float(data.get("y", 0.0)), float(data.get("yaw", 0.0))
        except Exception as exc:
            print(f"[API] pose fetch failed: {exc}")
            return None

    def follow_path_point(
        self,
        x: float,
        y: float,
        speed_ratio: float = 1.0,
        acceptable_precision: float = 0.18,
        fail_retry_count: int = 1,
    ) -> Optional[str]:
        pose = self.get_pose()
        if pose is None:
            return None

        robot_x, robot_y, _robot_yaw = pose
        yaw_rad = math.atan2(y - robot_y, x - robot_x)
        action_data = {
            "action_name": "slamtec.agent.actions.FollowPathPointsAction",
            "options": {
                "path_points": [
                    {"x": round(x, 3), "y": round(y, 3), "z": 0},
                ],
                "move_options": {"mode": 0, "flags": ["precise", "with_yaw"]},
                "yaw": round(yaw_rad, 6),
                "acceptable_precision": round(acceptable_precision, 3),
                "fail_retry_count": int(fail_retry_count),
                "speed_ratio": round(speed_ratio, 3),
            },
        }
        try:
            resp = SESSION.post(f"{self.base}/api/core/motion/v1/actions", headers=HEADERS, json=action_data, timeout=10)
            resp.raise_for_status()
            action_id = resp.json().get("action_id")
            if action_id:
                print(
                    f"[dispatch] FollowPathPointsAction -> ({x:.3f}, {y:.3f}) "
                    f"yaw={math.degrees(yaw_rad):.1f}deg action_id={action_id[:8]}..."
                )
                return action_id
            print(f"[dispatch] no action_id: {resp.text}")
            return None
        except requests.exceptions.RequestException as exc:
            print(f"[dispatch] request failed: {exc}")
            return None

    def follow_leg_person(
        self,
        person: Dict[str, Any],
        speed_ratio: float = 1.0,
        acceptable_precision: float = 0.18,
        fail_retry_count: int = 1,
    ) -> Optional[str]:
        """
        Convert a detected single person from robot-local leg coordinates into map coordinates,
        then dispatch a single-point FollowPathPointsAction.
        """
        pose = self.get_pose()
        if pose is None:
            print("[dispatch] skip: pose unavailable")
            return None

        robot_x, robot_y, robot_yaw = pose
        # person coordinates are in robot body frame:
        # +x = forward, +y = left
        forward_m = float(person["mid_x_m"])
        left_m = float(person["mid_y_m"])
        map_x = robot_x + left_m * math.cos(robot_yaw) - forward_m * math.sin(robot_yaw)
        map_y = robot_y + left_m * math.sin(robot_yaw) + forward_m * math.cos(robot_yaw)
        return self.follow_path_point(
            x=map_x,
            y=map_y,
            speed_ratio=speed_ratio,
            acceptable_precision=acceptable_precision,
            fail_retry_count=fail_retry_count,
        )


def run_once(
    ip: str,
    output_json: bool,
    api: Optional[RobotAPI],
    auto_dispatch: bool,
    dispatch_speed_ratio: float,
    dispatch_precision: float,
    dispatch_retry: int,
) -> None:
    data, t_req_start, t_req_end = fetch_laserscan(ip)
    t_ana_start = datetime.now()
    legs, people = detect_legs(data)
    people = filter_people(people)
    t_ana_end = datetime.now()
    timing = {
        "t_request_start": t_req_start,
        "t_request_end": t_req_end,
        "t_analysis_start": t_ana_start,
        "t_analysis_end": t_ana_end,
    }
    if output_json:
        print_json_output(data, timing)
    else:
        print_result(data, timing)

    if api and auto_dispatch:
        if len(people) == 1:
            person = people[0]
            print(
                f"[dispatch] single person -> d={person['d_m']:.3f}m "
                f"theta={person['theta_deg']:.1f}deg"
            )
            api.follow_leg_person(
                person=person,
                speed_ratio=dispatch_speed_ratio,
                acceptable_precision=dispatch_precision,
                fail_retry_count=dispatch_retry,
            )
        else:
            print(f"[dispatch] skip: person_count={len(people)}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Leg detection test tool with optional FollowPathPointsAction dispatch."
    )
    parser.add_argument("ip", nargs="?", default=ROBOT_IP, help=f"robot ip (default: {ROBOT_IP})")
    parser.add_argument("--once", "-o", action="store_true", help="fetch only once")
    parser.add_argument("--interval", "-i", type=float, default=DEFAULT_INTERVAL, help=f"poll interval in seconds")
    parser.add_argument("--json", "-j", action="store_true", help="print json output")
    parser.add_argument("--auto-dispatch", action="store_true", help="dispatch only when exactly one person is detected")
    parser.add_argument("--dispatch-speed-ratio", type=float, default=1.0, help="follow speed ratio")
    parser.add_argument("--dispatch-precision", type=float, default=0.18, help="acceptable precision in meters")
    parser.add_argument("--dispatch-retry", type=int, default=1, help="fail retry count")
    args = parser.parse_args()

    api = RobotAPI(args.ip) if args.auto_dispatch else None

    print(f"Leg test target: http://{args.ip}:{ROBOT_PORT}/api/core/system/v1/laserscan")
    if args.auto_dispatch:
        print("Auto dispatch: enabled (single-person only)")

    if args.once:
        try:
            run_once(
                ip=args.ip,
                output_json=args.json,
                api=api,
                auto_dispatch=args.auto_dispatch,
                dispatch_speed_ratio=args.dispatch_speed_ratio,
                dispatch_precision=args.dispatch_precision,
                dispatch_retry=args.dispatch_retry,
            )
        except Exception as exc:
            print(f"error: {exc}")
            traceback.print_exc()
        return

    try:
        while True:
            try:
                run_once(
                    ip=args.ip,
                    output_json=args.json,
                    api=api,
                    auto_dispatch=args.auto_dispatch,
                    dispatch_speed_ratio=args.dispatch_speed_ratio,
                    dispatch_precision=args.dispatch_precision,
                    dispatch_retry=args.dispatch_retry,
                )
            except Exception as exc:
                print(f"\nloop error: {exc}")
                traceback.print_exc()
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nbye")


if __name__ == "__main__":
    main()
