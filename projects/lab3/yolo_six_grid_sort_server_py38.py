#!/usr/bin/env python3
"""Python 3.8 vision/control server for the six-cell Coke/Sprite sorting task."""

from __future__ import annotations

import argparse
import csv
import socket
import sys
import time
from collections import deque
from pathlib import Path

from robomaster_split_protocol import recv_packet, send_packet


WINDOW_NAME = "RoboMaster camera - Coke/Sprite YOLO (Q/Esc to quit)"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--confidence", type=float, default=0.30)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--labels", default="sprite_can,coke_can")
    parser.add_argument(
        "--target-sequence",
        default="",
        help="Comma-separated labels forced at the start of the mission",
    )
    parser.add_argument("--target-x", type=float, default=0.50)
    parser.add_argument(
        "--align-deadband",
        type=float,
        default=0.06,
        help="Half width of the centered band as a fraction of image width",
    )
    parser.add_argument("--record", type=Path)
    parser.add_argument("--accept-timeout", type=float, default=120.0)
    parser.add_argument(
        "--control-mode",
        choices=("preview", "strafe-calibration", "auto-align", "auto-approach"),
        default="preview",
        help="Preview is motion-free; other modes enable guarded wheel pulses",
    )
    parser.add_argument("--stable-frames", type=int, default=3)
    parser.add_argument("--lost-target-frames", type=int, default=5)
    parser.add_argument("--max-auto-pulses", type=int, default=20)
    parser.add_argument("--pid-kp", type=float, default=2.2)
    parser.add_argument("--pid-ki", type=float, default=0.05)
    parser.add_argument("--pid-kd", type=float, default=0.02)
    parser.add_argument("--pid-integral-limit", type=float, default=0.30)
    parser.add_argument("--pid-min-pulse", type=float, default=0.08)
    parser.add_argument("--pid-max-pulse", type=float, default=0.35)
    parser.add_argument("--approach-stop-center-y", type=float, default=0.530)
    parser.add_argument("--approach-hard-stop-center-y", type=float, default=0.540)
    parser.add_argument("--near-field-center-y", type=float, default=0.46)
    parser.add_argument("--near-align-tolerance", type=float, default=0.05)
    parser.add_argument("--final-grasp-zone-center-y", type=float, default=0.48)
    parser.add_argument("--final-grasp-tolerance", type=float, default=0.07)
    parser.add_argument("--fallback-grasp-center-y", type=float, default=0.510)
    parser.add_argument("--fallback-grasp-tolerance", type=float, default=0.085)
    parser.add_argument("--fallback-grasp-min-height", type=float, default=0.54)
    parser.add_argument("--stall-grasp-center-y", type=float, default=0.510)
    parser.add_argument("--stall-grasp-min-height", type=float, default=0.54)
    parser.add_argument("--stall-progress-epsilon", type=float, default=0.004)
    parser.add_argument("--stall-forward-pulses", type=int, default=3)
    parser.add_argument("--push-plateau-frames", type=int, default=5)
    parser.add_argument("--near-lateral-pulse", type=float, default=0.10)
    parser.add_argument("--near-lateral-rpm", type=float, default=20.0)
    parser.add_argument("--max-near-align-pulses", type=int, default=3)
    parser.add_argument("--absolute-stop-center-y", type=float, default=0.560)
    parser.add_argument("--approach-min-height", type=float, default=0.08)
    parser.add_argument("--distance-filter-frames", type=int, default=5)
    parser.add_argument("--approach-kp", type=float, default=1.0)
    parser.add_argument("--approach-min-pulse", type=float, default=0.08)
    parser.add_argument("--approach-max-pulse", type=float, default=0.25)
    parser.add_argument("--max-forward-pulses", type=int, default=20)
    parser.add_argument(
        "--log-center-y-every-frame",
        action="store_true",
        help="Print raw and filtered target center-Y for every processed frame",
    )
    parser.add_argument("--center-y-log", type=Path)
    parser.add_argument("--steer-kp", type=float, default=100.0)
    parser.add_argument("--steer-ki", type=float, default=2.0)
    parser.add_argument("--steer-kd", type=float, default=0.2)
    parser.add_argument("--steer-max-rpm", type=float, default=10.0)
    parser.add_argument("--steer-recovery-error", type=float, default=0.12)
    parser.add_argument("--max-sort-items", type=int, default=6)
    parser.add_argument("--empty-finish-frames", type=int, default=10)
    parser.add_argument("--cycle-rearm-frames", type=int, default=3)
    parser.add_argument("--cycle-rearm-max-center-y", type=float, default=0.35)
    parser.add_argument("--search-forward-after-frames", type=int, default=10)
    parser.add_argument("--search-forward-distance", type=float, default=0.10)
    parser.add_argument("--max-search-forward-steps", type=int, default=10)
    parser.add_argument(
        "--target-policy",
        choices=("largest", "front-left"),
        default="largest",
    )
    return parser.parse_args()


def choose_target(
    result,
    allowed_labels,
    locked_target=None,
    target_policy="largest",
    max_center_y_ratio=None,
):
    candidates = []
    if result.boxes is None:
        return None
    for box in result.boxes:
        class_id = int(box.cls[0].item())
        label = str(result.names[class_id]).lower()
        if label not in allowed_labels:
            continue
        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0].tolist())
        item = {
            "label": label,
            "confidence": float(box.conf[0].item()),
            "x1": x1,
            "y1": y1,
            "x2": x2,
            "y2": y2,
            "center_x_px": 0.5 * (x1 + x2),
            "center_y_px": 0.5 * (y1 + y2),
        }
        if (
            max_center_y_ratio is not None
            and item["center_y_px"] / float(result.orig_shape[0])
            > max_center_y_ratio
        ):
            continue
        area = max(0, x2 - x1) * max(0, y2 - y1)
        item["area_px"] = area
        candidates.append(item)
    if not candidates:
        return None
    if locked_target is None:
        if target_policy == "front-left":
            # Perspective layout: the front row appears lower in the image.
            # Keep at most the three lowest cans, then choose the leftmost.
            front_candidates = sorted(
                candidates,
                key=lambda item: (item["center_y_px"], item["area_px"]),
                reverse=True,
            )[:3]
            return min(front_candidates, key=lambda item: item["center_x_px"])
        return max(candidates, key=lambda item: (item["area_px"], item["confidence"]))
    same_label = [
        item for item in candidates if item["label"] == locked_target["label"]
    ]
    if not same_label:
        return None
    nearest = min(
        same_label,
        key=lambda item: (
            (item["center_x_px"] - locked_target["center_x_px"]) ** 2
            + (item["center_y_px"] - locked_target["center_y_px"]) ** 2
        ),
    )
    frame_width = float(result.orig_shape[1])
    max_jump = 0.25 * frame_width
    jump = (
        (nearest["center_x_px"] - locked_target["center_x_px"]) ** 2
        + (nearest["center_y_px"] - locked_target["center_y_px"]) ** 2
    ) ** 0.5
    return nearest if jump <= max_jump else None


class PIDPulseController:
    """Convert normalized horizontal error into a bounded pulse duration."""

    def __init__(self, kp, ki, kd, integral_limit, min_pulse, max_pulse):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.min_pulse = min_pulse
        self.max_pulse = max_pulse
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_error = None
        self.last_time = None

    def update(self, error, now):
        derivative = 0.0
        if self.last_time is not None:
            dt = max(now - self.last_time, 1e-3)
            self.integral += error * dt
            self.integral = max(
                -self.integral_limit, min(self.integral, self.integral_limit)
            )
            derivative = (error - self.last_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        # Noisy derivative data must never reverse the direction selected by
        # the current image error.
        if output * error <= 0:
            output = self.kp * error
        duration = max(self.min_pulse, min(abs(output), self.max_pulse))
        self.last_error = error
        self.last_time = now
        return duration, output


class SignedPIDController:
    """Bounded signed PID output used as lateral RPM while moving forward."""

    def __init__(self, kp, ki, kd, max_output, integral_limit=0.30):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = abs(max_output)
        self.integral_limit = abs(integral_limit)
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_error = None
        self.last_time = None

    def update(self, error, now):
        derivative = 0.0
        if self.last_time is not None:
            dt = max(now - self.last_time, 1e-3)
            self.integral += error * dt
            self.integral = max(
                -self.integral_limit, min(self.integral, self.integral_limit)
            )
            derivative = (error - self.last_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if output * error <= 0:
            output = self.kp * error
        output = max(-self.max_output, min(output, self.max_output))
        self.last_error = error
        self.last_time = now
        return output


def approach_pulse_duration(height_ratio, stop_height, kp, min_pulse, max_pulse):
    remaining = max(stop_height - height_ratio, 0.0)
    return max(min_pulse, min(kp * remaining, max_pulse))


def main():
    args = parse_args()
    if args.stable_frames < 1:
        raise ValueError("--stable-frames must be at least 1")
    if args.lost_target_frames < 1:
        raise ValueError("--lost-target-frames must be at least 1")
    if args.max_auto_pulses < 1:
        raise ValueError("--max-auto-pulses must be at least 1")
    if min(args.pid_kp, args.pid_ki, args.pid_kd) < 0:
        raise ValueError("PID gains must not be negative")
    if not 0.05 <= args.pid_min_pulse <= args.pid_max_pulse <= 1.0:
        raise ValueError("PID pulse limits must satisfy 0.05 <= min <= max <= 1.0")
    if not 0.10 <= args.approach_stop_center_y <= 0.95:
        raise ValueError("--approach-stop-center-y must be between 0.10 and 0.95")
    if not args.approach_stop_center_y <= args.approach_hard_stop_center_y <= 0.95:
        raise ValueError("hard stop center-Y must be >= filtered stop center-Y")
    if not 0.10 <= args.near_field_center_y < args.approach_hard_stop_center_y:
        raise ValueError("near-field center-Y must be below hard stop center-Y")
    if not args.align_deadband <= args.near_align_tolerance < 0.20:
        raise ValueError("near alignment tolerance must be >= alignment deadband")
    if not args.near_field_center_y <= args.final_grasp_zone_center_y < args.approach_hard_stop_center_y:
        raise ValueError("final grasp zone must be between near field and hard stop")
    if not args.near_align_tolerance <= args.final_grasp_tolerance < 0.20:
        raise ValueError("final grasp tolerance must be >= near alignment tolerance")
    if not args.final_grasp_zone_center_y <= args.fallback_grasp_center_y < args.approach_hard_stop_center_y:
        raise ValueError("fallback grasp center-Y must be inside the final grasp zone")
    if not args.final_grasp_tolerance <= args.fallback_grasp_tolerance < 0.20:
        raise ValueError("fallback grasp tolerance must be >= final grasp tolerance")
    if not args.approach_min_height <= args.fallback_grasp_min_height <= 0.95:
        raise ValueError("fallback grasp height must be >= approach minimum height")
    if not args.final_grasp_zone_center_y <= args.stall_grasp_center_y <= args.approach_stop_center_y:
        raise ValueError("stall grasp center-Y must be inside the final approach zone")
    if not args.approach_min_height <= args.stall_grasp_min_height <= 0.95:
        raise ValueError("stall grasp height must be >= approach minimum height")
    if not 0 < args.stall_progress_epsilon <= 0.05:
        raise ValueError("stall progress epsilon must be between 0 and 0.05")
    if args.stall_forward_pulses < 2:
        raise ValueError("stall forward pulses must be at least 2")
    if not 3 <= args.push_plateau_frames <= 15:
        raise ValueError("push plateau frames must be between 3 and 15")
    if not 0.05 <= args.near_lateral_pulse <= 0.15:
        raise ValueError("near lateral pulse must be between 0.05 and 0.15 seconds")
    if not 1 <= args.near_lateral_rpm <= 100:
        raise ValueError("near lateral RPM must be between 1 and 100")
    if args.max_near_align_pulses < 1:
        raise ValueError("max near alignment pulses must be at least 1")
    if not args.approach_hard_stop_center_y < args.absolute_stop_center_y <= 0.95:
        raise ValueError("absolute stop center-Y must be above hard stop center-Y")
    if not 0.01 <= args.approach_min_height <= 0.95:
        raise ValueError("--approach-min-height must be between 0.01 and 0.95")
    if not 3 <= args.distance_filter_frames <= 15:
        raise ValueError("--distance-filter-frames must be between 3 and 15")
    if args.approach_kp <= 0:
        raise ValueError("--approach-kp must be positive")
    if not 0.05 <= args.approach_min_pulse <= args.approach_max_pulse <= 1.0:
        raise ValueError("approach pulse limits must satisfy 0.05 <= min <= max <= 1.0")
    if args.max_forward_pulses < 1:
        raise ValueError("--max-forward-pulses must be at least 1")
    if min(args.steer_kp, args.steer_ki, args.steer_kd) < 0:
        raise ValueError("steering PID gains must not be negative")
    if not 0 < args.steer_max_rpm <= 100:
        raise ValueError("--steer-max-rpm must be between 0 and 100")
    if not args.align_deadband < args.steer_recovery_error < 0.5:
        raise ValueError("recovery error must be larger than the alignment deadband")
    if not 1 <= args.max_sort_items <= 6:
        raise ValueError("max sort items must be between 1 and 6")
    if args.empty_finish_frames < args.lost_target_frames:
        raise ValueError("empty finish frames must be >= lost target frames")
    if not 2 <= args.cycle_rearm_frames <= 30:
        raise ValueError("cycle rearm frames must be between 2 and 30")
    if not 0.05 <= args.cycle_rearm_max_center_y < args.near_field_center_y:
        raise ValueError("cycle rearm center-Y must be below near-field center-Y")
    if not 1 <= args.search_forward_after_frames <= 60:
        raise ValueError("search forward delay must be between 1 and 60 frames")
    if not 0.02 <= args.search_forward_distance <= 0.30:
        raise ValueError("search forward distance must be between 0.02 and 0.30 m")
    if not 1 <= args.max_search_forward_steps <= 30:
        raise ValueError("max search forward steps must be between 1 and 30")
    if not args.model.is_file():
        print("找不到模型：%s" % args.model, file=sys.stderr)
        return 2

    try:
        import cv2
        import numpy as np
        import torch
        from ultralytics import YOLO
    except ImportError as exc:
        print("视觉环境缺少依赖：%s" % exc, file=sys.stderr)
        return 2

    print("Python：%s" % sys.executable)
    print("Torch：%s，CUDA=%s" % (torch.__version__, torch.cuda.is_available()))
    print("加载模型：%s" % args.model)
    model = YOLO(str(args.model))
    print("模型类别：%s" % model.names)
    allowed_labels = {v.strip().lower() for v in args.labels.split(",") if v.strip()}
    target_sequence = [
        value.strip().lower()
        for value in args.target_sequence.split(",")
        if value.strip()
    ]
    unknown_sequence_labels = set(target_sequence) - allowed_labels
    if unknown_sequence_labels:
        raise ValueError(
            "目标顺序包含未启用类别：%s" % sorted(unknown_sequence_labels)
        )

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((args.host, args.port))
    listener.listen(1)
    listener.settimeout(args.accept_timeout)
    writer = None
    center_log_stream = None
    center_log_writer = None
    conn = None
    last_geometry_log = 0.0
    control_unlocked = False
    pulse_ready = True
    last_key_activity = 0.0
    stable_state = None
    stable_frames = 0
    lost_target_frames = 0
    auto_pulse_count = 0
    near_align_pulse_count = 0
    stall_best_center_y = None
    stall_no_progress_pulses = 0
    forward_pulse_count = 0
    height_history = deque(maxlen=args.distance_filter_frames)
    center_y_history = deque(maxlen=args.distance_filter_frames)
    push_plateau_history = deque(maxlen=args.push_plateau_frames)
    auto_status = "LOCKED"
    sorted_count = 0
    locked_target = None
    awaiting_cycle_rearm = False
    cycle_rearm_frames = 0
    search_no_target_frames = 0
    search_forward_steps = 0
    pid = PIDPulseController(
        args.pid_kp,
        args.pid_ki,
        args.pid_kd,
        args.pid_integral_limit,
        args.pid_min_pulse,
        args.pid_max_pulse,
    )
    steering_pid = SignedPIDController(
        args.steer_kp,
        args.steer_ki,
        args.steer_kd,
        args.steer_max_rpm,
    )
    try:
        if args.center_y_log is not None:
            args.center_y_log.parent.mkdir(parents=True, exist_ok=True)
            center_log_stream = args.center_y_log.open(
                "w", encoding="utf-8-sig", newline=""
            )
            center_log_writer = csv.writer(center_log_stream)
            center_log_writer.writerow(
                [
                    "frame_id",
                    "timestamp",
                    "detected",
                    "label",
                    "confidence",
                    "center_y_raw",
                    "center_y_filtered",
                    "distance_samples",
                    "height_ratio",
                    "horizontal_error_ratio",
                    "horizontal_state",
                    "command",
                    "pulse_duration_s",
                    "lateral_rpm",
                    "lateral_wheel_rpm",
                    "control_status",
                    "stall_no_progress_pulses",
                    "push_plateau_span",
                ]
            )
            center_log_stream.flush()
            print("逐帧数据将保存到：%s" % args.center_y_log.resolve())
        print("等待 RoboMaster 摄像头进程连接 %s:%d ..." % (args.host, args.port))
        conn, address = listener.accept()
        # A complete pick/place/return cycle can take tens of seconds.
        conn.settimeout(60.0)
        print("摄像头进程已连接：%s:%d" % address)

        while True:
            message, payload = recv_packet(conn)
            if message.get("type") == "cycle_complete":
                reported_cycle_id = int(message.get("cycle_id", sorted_count))
                if reported_cycle_id != sorted_count:
                    control_unlocked = False
                    send_packet(
                        conn,
                        {
                            "type": "result",
                            "command": "stop",
                            "control_unlocked": False,
                            "sorted_count": sorted_count,
                            "cycle_id": sorted_count,
                            "error": "cycle_id_mismatch",
                        },
                    )
                    continue
                sorted_count += 1
                stable_state = None
                stable_frames = 0
                lost_target_frames = 0
                auto_pulse_count = 0
                near_align_pulse_count = 0
                stall_best_center_y = None
                stall_no_progress_pulses = 0
                forward_pulse_count = 0
                height_history.clear()
                center_y_history.clear()
                push_plateau_history.clear()
                locked_target = None
                cycle_rearm_frames = 0
                search_no_target_frames = 0
                search_forward_steps = 0
                pid.reset()
                steering_pid.reset()
                if sorted_count >= args.max_sort_items:
                    control_unlocked = False
                    awaiting_cycle_rearm = False
                    auto_status = "SIX ITEMS SORTED - COMPLETE"
                    next_command = "mission_complete"
                else:
                    # Never carry a close/frozen detection directly into the
                    # next grasp cycle. Require a farther target to be visible
                    # for several new frames before motion is re-enabled.
                    control_unlocked = False
                    awaiting_cycle_rearm = True
                    auto_status = "WAITING FOR FRESH NEXT TARGET"
                    next_command = "cycle_ready"
                send_packet(
                    conn,
                    {
                        "type": "result",
                        "command": next_command,
                        "control_unlocked": control_unlocked,
                        "sorted_count": sorted_count,
                        "cycle_id": sorted_count,
                        "awaiting_cycle_rearm": awaiting_cycle_rearm,
                    },
                )
                continue
            if message.get("type") != "frame":
                continue
            if int(message.get("cycle_id", -1)) != sorted_count:
                control_unlocked = False
                send_packet(
                    conn,
                    {
                        "type": "result",
                        "frame_id": message.get("frame_id"),
                        "cycle_id": sorted_count,
                        "command": "stop",
                        "control_unlocked": False,
                        "error": "frame_cycle_id_mismatch",
                    },
                )
                continue
            frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                send_packet(
                    conn,
                    {
                        "type": "result",
                        "frame_id": message.get("frame_id"),
                        "cycle_id": sorted_count,
                        "command": "stop",
                        "control_unlocked": False,
                    },
                )
                continue

            started = time.perf_counter()
            result = model.predict(
                source=frame,
                conf=args.confidence,
                imgsz=args.imgsz,
                device=args.device,
                verbose=False,
            )[0]
            elapsed = max(time.perf_counter() - started, 1e-6)
            if sorted_count < len(target_sequence):
                active_allowed_labels = {target_sequence[sorted_count]}
                active_target_policy = (
                    args.target_policy if sorted_count == 0 else "largest"
                )
            else:
                active_allowed_labels = allowed_labels
                active_target_policy = "largest" if target_sequence else args.target_policy
            scene_target = choose_target(
                result,
                allowed_labels,
                None,
                "largest",
            )
            target = choose_target(
                result,
                active_allowed_labels,
                locked_target,
                active_target_policy,
                (
                    args.cycle_rearm_max_center_y
                    if awaiting_cycle_rearm
                    else None
                ),
            )
            if awaiting_cycle_rearm:
                if target is None:
                    cycle_rearm_frames = 0
                    if scene_target is None:
                        search_no_target_frames += 1
                        auto_status = "SEARCHING FOR TARGET %d" % search_no_target_frames
                    else:
                        search_no_target_frames = 0
                        auto_status = "OBJECT VISIBLE - SEARCH FORWARD STOPPED"
                else:
                    search_no_target_frames = 0
                    cycle_rearm_frames += 1
                    auto_status = "FRESH TARGET %d/%d" % (
                        cycle_rearm_frames,
                        args.cycle_rearm_frames,
                    )
                    if cycle_rearm_frames >= args.cycle_rearm_frames:
                        awaiting_cycle_rearm = False
                        control_unlocked = True
                        cycle_rearm_frames = 0
                        auto_status = "NEXT CYCLE ARMED"
            if control_unlocked and target is not None:
                locked_target = {
                    "label": target["label"],
                    "center_x_px": target["center_x_px"],
                    "center_y_px": target["center_y_px"],
                }
            # Ultralytics draws every detected Coke/Sprite box, class and confidence.
            annotated = result.plot(labels=True, conf=True, line_width=3)
            detection_count = 0 if result.boxes is None else len(result.boxes)

            # Geometry is always calculated here. Only guarded control modes may
            # translate it into a lateral command later in this loop.
            frame_height, frame_width = annotated.shape[:2]
            axis_x = int(args.target_x * frame_width)
            band_px = int(args.align_deadband * frame_width)
            left_limit = axis_x - band_px
            right_limit = axis_x + band_px
            cv2.line(annotated, (axis_x, 0), (axis_x, frame_height), (0, 255, 255), 2)
            cv2.line(annotated, (left_limit, 0), (left_limit, frame_height), (255, 180, 0), 1)
            cv2.line(annotated, (right_limit, 0), (right_limit, frame_height), (255, 180, 0), 1)

            geometry_text = "TARGET NOT FOUND"
            geometry_color = (0, 0, 255)
            if target is not None:
                target_center_x = (target["x1"] + target["x2"]) / 2.0
                target_center_y = (target["y1"] + target["y2"]) / 2.0
                error_px = target_center_x - axis_x
                error_ratio = target_center_x / float(frame_width) - args.target_x
                height_ratio = (target["y2"] - target["y1"]) / float(frame_height)
                center_y_ratio = target_center_y / float(frame_height)
                if target_center_x < left_limit:
                    horizontal_state = "TARGET LEFT"
                    geometry_color = (255, 180, 0)
                elif target_center_x > right_limit:
                    horizontal_state = "TARGET RIGHT"
                    geometry_color = (255, 180, 0)
                else:
                    horizontal_state = "CENTERED"
                    geometry_color = (0, 255, 0)

                target.update(
                    {
                        "center_x": target_center_x,
                        "center_y": target_center_y,
                        "error_px": error_px,
                        "error_ratio": error_ratio,
                        "height_ratio": height_ratio,
                        "center_y_ratio": center_y_ratio,
                        "horizontal_state": horizontal_state,
                    }
                )
                if args.control_mode == "auto-approach":
                    height_history.append(height_ratio)
                    center_y_history.append(center_y_ratio)
                    ordered_heights = sorted(height_history)
                    ordered_center_y = sorted(center_y_history)
                    target["distance_height_ratio"] = ordered_heights[
                        len(ordered_heights) // 2
                    ]
                    target["distance_center_y_ratio"] = ordered_center_y[
                        len(ordered_center_y) // 2
                    ]
                    target["distance_samples"] = len(center_y_history)
                cv2.circle(
                    annotated,
                    (int(target_center_x), int(target_center_y)),
                    7,
                    geometry_color,
                    -1,
                )
                cv2.line(
                    annotated,
                    (axis_x, int(target_center_y)),
                    (int(target_center_x), int(target_center_y)),
                    geometry_color,
                    2,
                )
                cv2.circle(
                    annotated,
                    (axis_x, int(target_center_y)),
                    5,
                    (255, 0, 255),
                    -1,
                )
                geometry_text = "%s | dx=%+.0fpx (%+.3f) | centerY=%.3f | h=%.3f" % (
                    horizontal_state,
                    error_px,
                    error_ratio,
                    center_y_ratio,
                    height_ratio,
                )
                now = time.monotonic()
                if not args.log_center_y_every_frame and now - last_geometry_log >= 0.5:
                    print(
                        "%s %.2f | %s"
                        % (target["label"], target["confidence"], geometry_text)
                    )
                    last_geometry_log = now
            elif args.control_mode == "auto-approach":
                height_history.clear()
                center_y_history.clear()

            if args.control_mode in ("auto-align", "auto-approach"):
                panel_height = 140
            elif args.control_mode == "strafe-calibration":
                panel_height = 108
            else:
                panel_height = 72
            cv2.rectangle(
                annotated, (0, 0), (frame_width, panel_height), (0, 0, 0), -1
            )
            if args.control_mode == "strafe-calibration":
                mode_text = "STRAFE CALIBRATION"
            elif args.control_mode == "auto-align":
                mode_text = "COKE AUTO ALIGN"
            elif args.control_mode == "auto-approach":
                mode_text = "COKE AUTO APPROACH"
            else:
                mode_text = "GEOMETRY ONLY - NO MOTION"
            cv2.putText(
                annotated,
                "FPS %.1f | boxes %d | %s | Q/Esc: quit"
                % (1.0 / elapsed, detection_count, mode_text),
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            cv2.putText(
                annotated,
                geometry_text,
                (12, 62),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.66,
                geometry_color,
                2,
            )
            if args.control_mode == "strafe-calibration":
                lock_text = "UNLOCKED" if control_unlocked else "LOCKED"
                ready_text = "READY" if pulse_ready else "RELEASE KEY"
                control_text = (
                    "%s | %s | S unlock | A/D single wheel pulse | X stop"
                    % (lock_text, ready_text)
                )
                cv2.putText(
                    annotated,
                    control_text,
                    (12, 94),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.56,
                    (0, 255, 0) if control_unlocked else (0, 180, 255),
                    2,
                )
            elif args.control_mode in ("auto-align", "auto-approach"):
                lock_text = "ARMED" if control_unlocked else "LOCKED"
                control_text = "%s | %s | lateral %d/%d | forward %d/%d" % (
                    lock_text,
                    auto_status,
                    auto_pulse_count,
                    args.max_auto_pulses,
                    forward_pulse_count,
                    args.max_forward_pulses,
                )
                cv2.putText(
                    annotated,
                    control_text,
                    (12, 94),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.56,
                    (0, 255, 0) if control_unlocked else (0, 180, 255),
                    2,
                )
                cv2.putText(
                    annotated,
                    "CONTROL LABEL: %s | completed %d/%d"
                    % (
                        ",".join(sorted(active_allowed_labels)),
                        sorted_count,
                        args.max_sort_items,
                    ),
                    (12, 126),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.56,
                    (255, 220, 0),
                    2,
                )

            if args.record:
                if writer is None:
                    args.record.parent.mkdir(parents=True, exist_ok=True)
                    writer = cv2.VideoWriter(
                        str(args.record),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        20.0,
                        (annotated.shape[1], annotated.shape[0]),
                    )
                writer.write(annotated)

            cv2.imshow(WINDOW_NAME, annotated)
            key = cv2.waitKey(1) & 0xFF
            window_closed = cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1
            now = time.monotonic()
            command = "continue"
            pulse_duration = None
            pid_output = None
            lateral_rpm = None
            lateral_wheel_rpm = None
            push_plateau_span = None
            if key != 255:
                last_key_activity = now
            elif not pulse_ready and now - last_key_activity >= 0.8:
                # Re-arm only after a quiet release interval. This prevents an
                # operating-system key repeat from becoming continuous motion.
                pulse_ready = True

            if key in (ord("q"), ord("Q"), 27) or window_closed:
                control_unlocked = False
                command = "quit"
            elif args.control_mode == "strafe-calibration":
                if key in (ord("x"), ord("X")):
                    control_unlocked = False
                    awaiting_cycle_rearm = False
                    cycle_rearm_frames = 0
                    search_no_target_frames = 0
                    search_forward_steps = 0
                    command = "stop"
                elif key in (ord("s"), ord("S")):
                    control_unlocked = True
                elif control_unlocked and pulse_ready and key in (ord("a"), ord("A")):
                    command = "pulse_negative_y"
                    pulse_ready = False
                elif control_unlocked and pulse_ready and key in (ord("d"), ord("D")):
                    command = "pulse_positive_y"
                    pulse_ready = False
            elif args.control_mode in ("auto-align", "auto-approach"):
                if key in (ord("x"), ord("X")):
                    control_unlocked = False
                    awaiting_cycle_rearm = False
                    cycle_rearm_frames = 0
                    search_no_target_frames = 0
                    search_forward_steps = 0
                    stable_state = None
                    stable_frames = 0
                    height_history.clear()
                    center_y_history.clear()
                    push_plateau_history.clear()
                    locked_target = None
                    stall_best_center_y = None
                    stall_no_progress_pulses = 0
                    auto_status = "STOPPED"
                    pid.reset()
                    steering_pid.reset()
                    command = "stop"
                elif key in (ord("s"), ord("S")):
                    if awaiting_cycle_rearm:
                        auto_status = "WAITING FOR FRESH NEXT TARGET"
                        command = "stop"
                    elif not control_unlocked:
                        stable_state = None
                        stable_frames = 0
                        lost_target_frames = 0
                        auto_pulse_count = 0
                        near_align_pulse_count = 0
                        stall_best_center_y = None
                        stall_no_progress_pulses = 0
                        forward_pulse_count = 0
                        height_history.clear()
                        center_y_history.clear()
                        push_plateau_history.clear()
                        locked_target = None
                        pid.reset()
                        steering_pid.reset()
                    if not awaiting_cycle_rearm:
                        control_unlocked = True
                        auto_status = "WAITING FOR STABLE TARGET"
                        search_no_target_frames = 0
                        search_forward_steps = 0
                elif awaiting_cycle_rearm:
                    if scene_target is not None:
                        command = "stop"
                    elif search_no_target_frames >= args.search_forward_after_frames:
                        if search_forward_steps >= args.max_search_forward_steps:
                            auto_status = "SEARCH FORWARD LIMIT - WAITING"
                            command = "stop"
                        else:
                            search_forward_steps += 1
                            auto_status = "SEARCH FORWARD %.2fM %d/%d" % (
                                args.search_forward_distance,
                                search_forward_steps,
                                args.max_search_forward_steps,
                            )
                            command = "search_forward_step"
                elif control_unlocked:
                    if target is None:
                        stable_state = None
                        stable_frames = 0
                        height_history.clear()
                        center_y_history.clear()
                        push_plateau_history.clear()
                        stall_best_center_y = None
                        stall_no_progress_pulses = 0
                        lost_target_frames += 1
                        auto_status = "TARGET LOST %d/%d" % (
                            lost_target_frames,
                            args.lost_target_frames,
                        )
                        if lost_target_frames >= args.lost_target_frames:
                            if (
                                sorted_count > 0
                                and lost_target_frames >= args.empty_finish_frames
                            ):
                                control_unlocked = False
                                auto_status = "NO TARGETS LEFT - COMPLETE"
                                pid.reset()
                                steering_pid.reset()
                                command = "mission_complete"
                            elif sorted_count > 0 and locked_target is not None:
                                # The previously placed can can remain visible for
                                # a few frames and become the next lock. If that
                                # object then leaves the camera, release the stale
                                # lock so another visible can may be selected.
                                locked_target = None
                                stable_state = None
                                stable_frames = 0
                                height_history.clear()
                                center_y_history.clear()
                                push_plateau_history.clear()
                                stall_best_center_y = None
                                stall_no_progress_pulses = 0
                                pid.reset()
                                steering_pid.reset()
                                control_unlocked = False
                                awaiting_cycle_rearm = True
                                cycle_rearm_frames = 0
                                search_no_target_frames = 0
                                search_forward_steps = 0
                                auto_status = "OLD TARGET LOST - SEARCH FORWARD"
                                command = "stop"
                            else:
                                control_unlocked = False
                                awaiting_cycle_rearm = True
                                cycle_rearm_frames = 0
                                search_no_target_frames = 0
                                search_forward_steps = 0
                                auto_status = "TARGET LOST - SEARCH FORWARD"
                                pid.reset()
                                steering_pid.reset()
                                command = "stop"
                    else:
                        lost_target_frames = 0
                        current_state = target["horizontal_state"]
                        error_ratio = float(target["error_ratio"])
                        height_ratio = float(target["height_ratio"])
                        filtered_height_ratio = float(
                            target.get("distance_height_ratio", height_ratio)
                        )
                        filtered_center_y_ratio = float(
                            target.get(
                                "distance_center_y_ratio",
                                target["center_y_ratio"],
                            )
                        )
                        raw_center_y_ratio = float(target["center_y_ratio"])
                        near_field = (
                            max(raw_center_y_ratio, filtered_center_y_ratio)
                            >= args.near_field_center_y
                        )
                        final_grasp_zone = (
                            max(raw_center_y_ratio, filtered_center_y_ratio)
                            >= args.final_grasp_zone_center_y
                        )
                        distance_metric = max(
                            raw_center_y_ratio,
                            filtered_center_y_ratio,
                        )
                        if final_grasp_zone:
                            push_plateau_history.append(distance_metric)
                        else:
                            push_plateau_history.clear()
                        if len(push_plateau_history) >= 2:
                            push_plateau_span = (
                                max(push_plateau_history)
                                - min(push_plateau_history)
                            )
                        active_grasp_tolerance = (
                            args.final_grasp_tolerance
                            if final_grasp_zone
                            else args.near_align_tolerance
                        )
                        near_aligned = abs(error_ratio) <= active_grasp_tolerance
                        fallback_grasp_safe = (
                            final_grasp_zone
                            and max(raw_center_y_ratio, filtered_center_y_ratio)
                            >= args.fallback_grasp_center_y
                            and abs(error_ratio) <= args.fallback_grasp_tolerance
                            and height_ratio >= args.fallback_grasp_min_height
                        )
                        stall_grasp_safe = (
                            final_grasp_zone
                            and max(raw_center_y_ratio, filtered_center_y_ratio)
                            >= args.stall_grasp_center_y
                            and abs(error_ratio) <= args.final_grasp_tolerance
                            and height_ratio >= args.stall_grasp_min_height
                        )
                        push_plateau_grasp_safe = (
                            stall_grasp_safe
                            and len(push_plateau_history)
                            >= args.push_plateau_frames
                            and push_plateau_span is not None
                            and push_plateau_span
                            <= args.stall_progress_epsilon
                        )
                        hard_stop = (
                            args.control_mode == "auto-approach"
                            and raw_center_y_ratio >= args.approach_hard_stop_center_y
                            and height_ratio >= args.approach_min_height
                        )
                        absolute_stop = (
                            args.control_mode == "auto-approach"
                            and raw_center_y_ratio >= args.absolute_stop_center_y
                            and height_ratio >= args.approach_min_height
                        )
                        if current_state == stable_state:
                            stable_frames += 1
                        else:
                            stable_state = current_state
                            stable_frames = 1
                        if hard_stop and near_aligned:
                            # Bypass median-filter delay near physical contact.
                            stable_frames = args.stable_frames
                        auto_status = "%s STABLE %d/%d" % (
                            current_state,
                            stable_frames,
                            args.stable_frames,
                        )
                        if stable_frames >= args.stable_frames:
                            stable_frames = 0
                            if args.control_mode == "auto-approach":
                                if (hard_stop and near_aligned) or (
                                    near_aligned
                                    and len(height_history)
                                    >= args.distance_filter_frames
                                    and filtered_center_y_ratio
                                    >= args.approach_stop_center_y
                                    and filtered_height_ratio
                                    >= args.approach_min_height
                                ):
                                    control_unlocked = False
                                    auto_status = (
                                        "HARD STOP - LOCKED"
                                        if hard_stop
                                        else "APPROACH COMPLETE - LOCKED"
                                    )
                                    pid.reset()
                                    steering_pid.reset()
                                    command = "approach_complete"
                                elif push_plateau_grasp_safe:
                                    control_unlocked = False
                                    auto_status = "PUSH PLATEAU - GRASPING"
                                    pid.reset()
                                    steering_pid.reset()
                                    command = "approach_complete"
                                elif (
                                    stall_grasp_safe
                                    and stall_no_progress_pulses
                                    >= args.stall_forward_pulses
                                ):
                                    control_unlocked = False
                                    auto_status = "DISTANCE STALLED - GRASPING"
                                    pid.reset()
                                    steering_pid.reset()
                                    command = "approach_complete"
                                elif fallback_grasp_safe and (
                                    near_align_pulse_count
                                    >= args.max_near_align_pulses
                                    or forward_pulse_count
                                    >= args.max_forward_pulses
                                ):
                                    # A tilted/light can can move with the chassis,
                                    # leaving center-Y almost unchanged. Once guarded
                                    # correction limits are exhausted and the can is
                                    # already inside the gripper envelope, stop trying
                                    # to perfect the image geometry and grasp it.
                                    control_unlocked = False
                                    auto_status = "FALLBACK GRASP - LOCKED"
                                    pid.reset()
                                    steering_pid.reset()
                                    command = "approach_complete"
                                elif absolute_stop:
                                    control_unlocked = False
                                    auto_status = "TOO CLOSE / OFF-CENTER - LOCKED"
                                    pid.reset()
                                    steering_pid.reset()
                                    command = "approach_blocked"
                                elif near_field and not near_aligned:
                                    if near_align_pulse_count >= args.max_near_align_pulses:
                                        if abs(error_ratio) > args.fallback_grasp_tolerance:
                                            control_unlocked = False
                                            auto_status = "OUTSIDE GRIP ENVELOPE - LOCKED"
                                            pid.reset()
                                            steering_pid.reset()
                                            command = "approach_blocked"
                                        elif forward_pulse_count >= args.max_forward_pulses:
                                            control_unlocked = False
                                            auto_status = "FORWARD LIMIT BEFORE SAFE GRASP - LOCKED"
                                            pid.reset()
                                            steering_pid.reset()
                                            command = "approach_blocked"
                                        else:
                                            # Lateral retries are exhausted, but this
                                            # is not permission to grasp early. Freeze
                                            # strafe and continue straight until the
                                            # independent distance and box-height gates
                                            # make fallback_grasp_safe true.
                                            pulse_duration = approach_pulse_duration(
                                                max(
                                                    filtered_center_y_ratio,
                                                    raw_center_y_ratio,
                                                ),
                                                args.approach_stop_center_y,
                                                args.approach_kp,
                                                args.approach_min_pulse,
                                                args.approach_max_pulse,
                                            )
                                            lateral_rpm = 0.0
                                            command = "auto_drive_approach"
                                            forward_pulse_count += 1
                                            current_distance = max(
                                                raw_center_y_ratio,
                                                filtered_center_y_ratio,
                                            )
                                            if (
                                                stall_best_center_y is None
                                                or current_distance
                                                > stall_best_center_y
                                                + args.stall_progress_epsilon
                                            ):
                                                stall_best_center_y = current_distance
                                                stall_no_progress_pulses = 0
                                            else:
                                                stall_no_progress_pulses += 1
                                            pid.reset()
                                            steering_pid.reset()
                                            auto_status = "LATERAL FROZEN - SEEK SAFE DEPTH"
                                    else:
                                        # Close to the can, never combine forward and
                                        # strafe. Use a fixed, low-speed micro pulse to
                                        # avoid the large PID recovery that caused the
                                        # final left/right oscillation.
                                        pulse_duration = args.near_lateral_pulse
                                        lateral_wheel_rpm = args.near_lateral_rpm
                                        command = (
                                            "auto_pulse_left"
                                            if error_ratio < 0
                                            else "auto_pulse_right"
                                        )
                                        near_align_pulse_count += 1
                                        auto_pulse_count += 1
                                        stall_best_center_y = None
                                        stall_no_progress_pulses = 0
                                        push_plateau_history.clear()
                                        pid.reset()
                                        steering_pid.reset()
                                        auto_status = "NEAR MICRO ALIGN %d/%d" % (
                                            near_align_pulse_count,
                                            args.max_near_align_pulses,
                                        )
                                elif forward_pulse_count >= args.max_forward_pulses:
                                    control_unlocked = False
                                    auto_status = "FORWARD LIMIT - LOCKED"
                                    pid.reset()
                                    steering_pid.reset()
                                    command = "stop"
                                elif (
                                    (
                                        len(height_history)
                                        < args.distance_filter_frames
                                        or filtered_center_y_ratio
                                        < args.approach_stop_center_y
                                        or filtered_height_ratio
                                        < args.approach_min_height
                                    )
                                    and abs(error_ratio) <= args.steer_recovery_error
                                ):
                                    pulse_duration = approach_pulse_duration(
                                        max(
                                            filtered_center_y_ratio,
                                            float(target["center_y_ratio"]),
                                        ),
                                        args.approach_stop_center_y,
                                        args.approach_kp,
                                        args.approach_min_pulse,
                                        args.approach_max_pulse,
                                    )
                                    # Once in the near field, advance straight only.
                                    # Lateral correction is handled separately above.
                                    lateral_rpm = 0.0 if near_field else (
                                        0.0 if current_state == "CENTERED"
                                        else steering_pid.update(error_ratio, now)
                                    )
                                    command = "auto_drive_approach"
                                    forward_pulse_count += 1
                                    current_distance = max(
                                        raw_center_y_ratio,
                                        filtered_center_y_ratio,
                                    )
                                    if (
                                        stall_best_center_y is None
                                        or current_distance
                                        > stall_best_center_y
                                        + args.stall_progress_epsilon
                                    ):
                                        stall_best_center_y = current_distance
                                        stall_no_progress_pulses = 0
                                    else:
                                        stall_no_progress_pulses += 1
                                    auto_status = "DRIVE %.2fs Y%+.1f" % (
                                        pulse_duration,
                                        lateral_rpm,
                                    )
                                elif auto_pulse_count >= args.max_auto_pulses:
                                    control_unlocked = False
                                    auto_status = "RECOVERY LIMIT - LOCKED"
                                    pid.reset()
                                    steering_pid.reset()
                                    command = "stop"
                                else:
                                    pulse_duration, pid_output = pid.update(
                                        error_ratio, now
                                    )
                                    command = (
                                        "auto_pulse_left"
                                        if error_ratio < 0
                                        else "auto_pulse_right"
                                    )
                                    auto_pulse_count += 1
                                    auto_status = "RECOVER %.2fs" % pulse_duration
                            elif current_state == "CENTERED":
                                control_unlocked = False
                                auto_status = "ALIGNED - LOCKED"
                                pid.reset()
                                steering_pid.reset()
                                command = "aligned"
                            elif auto_pulse_count >= args.max_auto_pulses:
                                control_unlocked = False
                                auto_status = "PULSE LIMIT - LOCKED"
                                pid.reset()
                                steering_pid.reset()
                                command = "stop"
                            elif current_state == "TARGET LEFT":
                                pulse_duration, pid_output = pid.update(
                                    error_ratio, now
                                )
                                command = "auto_pulse_left"
                                auto_pulse_count += 1
                                auto_status = "PID LEFT %.2fs" % pulse_duration
                            elif current_state == "TARGET RIGHT":
                                pulse_duration, pid_output = pid.update(
                                    error_ratio, now
                                )
                                command = "auto_pulse_right"
                                auto_pulse_count += 1
                                auto_status = "PID RIGHT %.2fs" % pulse_duration
            if args.log_center_y_every_frame:
                frame_id = message.get("frame_id")
                if target is None:
                    print("FRAME %s | centerY=NA | target=NOT_FOUND" % frame_id, flush=True)
                else:
                    raw_center_y = float(target["center_y_ratio"])
                    filtered_center_y = target.get("distance_center_y_ratio")
                    samples = int(target.get("distance_samples", 0))
                    filtered_text = (
                        "NA"
                        if filtered_center_y is None
                        else "%.4f" % float(filtered_center_y)
                    )
                    print(
                        "FRAME %s | centerY=%.4f | filtered=%s | samples=%d | height=%.4f"
                        % (
                            frame_id,
                            raw_center_y,
                            filtered_text,
                            samples,
                            float(target["height_ratio"]),
                        ),
                        flush=True,
                    )
            if center_log_writer is not None:
                center_log_writer.writerow(
                    [
                        message.get("frame_id"),
                        "%.6f" % time.time(),
                        int(target is not None),
                        "" if target is None else target.get("label", ""),
                        "" if target is None else "%.6f" % target.get("confidence", 0.0),
                        "" if target is None else "%.6f" % target.get("center_y_ratio", 0.0),
                        ""
                        if target is None or target.get("distance_center_y_ratio") is None
                        else "%.6f" % target["distance_center_y_ratio"],
                        0 if target is None else target.get("distance_samples", 0),
                        "" if target is None else "%.6f" % target.get("height_ratio", 0.0),
                        "" if target is None else "%.6f" % target.get("error_ratio", 0.0),
                        "" if target is None else target.get("horizontal_state", ""),
                        command,
                        "" if pulse_duration is None else "%.6f" % pulse_duration,
                        "" if lateral_rpm is None else "%.6f" % lateral_rpm,
                        ""
                        if lateral_wheel_rpm is None
                        else "%.6f" % lateral_wheel_rpm,
                        auto_status,
                        stall_no_progress_pulses,
                        ""
                        if push_plateau_span is None
                        else "%.6f" % push_plateau_span,
                    ]
                )
                center_log_stream.flush()
            send_packet(
                conn,
                {
                    "type": "result",
                    "frame_id": message.get("frame_id"),
                    "command": command,
                    "control_unlocked": control_unlocked,
                    "target": target,
                    "detection_count": detection_count,
                    "fps": 1.0 / elapsed,
                    "pulse_duration": pulse_duration,
                    "search_distance": args.search_forward_distance,
                    "pid_output": pid_output,
                    "forward_pulse_count": forward_pulse_count,
                    "lateral_rpm": lateral_rpm,
                    "wheel_rpm": lateral_wheel_rpm,
                    "stall_no_progress_pulses": stall_no_progress_pulses,
                    "push_plateau_span": push_plateau_span,
                    "sorted_count": sorted_count,
                    "cycle_id": sorted_count,
                    "awaiting_cycle_rearm": awaiting_cycle_rearm,
                },
            )
            if command == "quit":
                break
        return 0
    except KeyboardInterrupt:
        return 130
    except (ConnectionError, socket.timeout) as exc:
        print("连接结束：%s" % exc, file=sys.stderr)
        return 1
    finally:
        if writer is not None:
            writer.release()
        if center_log_stream is not None:
            center_log_stream.close()
        cv2.destroyAllWindows()
        if conn is not None:
            conn.close()
        listener.close()


if __name__ == "__main__":
    raise SystemExit(main())
