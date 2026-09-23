#!/usr/bin/env python3
"""Python 3.7 RoboMaster client for six-cell Coke/Sprite sorting."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
import zlib
from datetime import datetime
from pathlib import Path

from robomaster_split_protocol import recv_packet, send_packet


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--conn-type", choices=("ap", "sta"), default="ap")
    parser.add_argument("--robot-ip", default="192.168.2.1")
    parser.add_argument("--jpeg-quality", type=int, default=85)
    parser.add_argument("--connect-timeout", type=float, default=90.0)
    parser.add_argument("--max-missing-camera-frames", type=int, default=3)
    parser.add_argument("--max-identical-camera-frames", type=int, default=8)
    parser.add_argument("--camera-freeze-timeout", type=float, default=2.0)
    parser.add_argument(
        "--safety-log",
        type=Path,
        default=Path("robomaster_client_safety.log"),
    )
    parser.add_argument(
        "--control-mode",
        choices=("preview", "strafe-calibration", "auto-align", "auto-approach"),
        default="preview",
    )
    parser.add_argument("--wheel-rpm", type=int, default=20)
    parser.add_argument(
        "--yaw-trim-rpm",
        type=int,
        default=0,
        help="Symmetric wheel trim that opposes strafe-induced yaw",
    )
    parser.add_argument("--pulse-duration", type=float, default=0.35)
    parser.add_argument("--soft-stop-rpm", type=int, default=0)
    parser.add_argument("--soft-stop-duration", type=float, default=0.0)
    parser.add_argument("--max-auto-pulses", type=int, default=20)
    parser.add_argument("--forward-wheel-rpm", type=int, default=22)
    parser.add_argument("--max-steer-rpm", type=int, default=10)
    parser.add_argument("--max-forward-pulses", type=int, default=20)
    parser.add_argument("--heading-tolerance-deg", type=float, default=1.0)
    parser.add_argument("--heading-max-correction-deg", type=float, default=3.0)
    parser.add_argument("--heading-correction-speed", type=float, default=10.0)
    parser.add_argument("--heading-settle-time", type=float, default=0.10)
    parser.add_argument("--grasp-after-approach", action="store_true")
    parser.add_argument("--arm-x-mm", type=int, default=180)
    parser.add_argument("--arm-ready-y-mm", type=int, default=40)
    parser.add_argument("--arm-lower-y-mm", type=int, default=0)
    parser.add_argument("--arm-lift-y-mm", type=int, default=80)
    parser.add_argument("--arm-action-timeout", type=float, default=15.0)
    parser.add_argument("--gripper-wait", type=float, default=1.5)
    parser.add_argument("--pre-grasp-settle", type=float, default=0.5)
    parser.add_argument("--sort-mission", action="store_true")
    parser.add_argument("--drop-lateral-distance", type=float, default=0.30)
    parser.add_argument("--sort-move-speed", type=float, default=0.6)
    parser.add_argument("--sort-action-timeout", type=float, default=12.0)
    parser.add_argument("--first-left-drop-test", action="store_true")
    parser.add_argument("--fixed-left-turn-deg", type=float, default=90.0)
    parser.add_argument("--fixed-return-turn-deg", type=float, default=100.0)
    parser.add_argument("--fixed-drop-forward-distance", type=float, default=0.30)
    parser.add_argument("--fixed-drop-retreat-distance", type=float, default=0.30)
    parser.add_argument("--sprite-drop-forward-distance", type=float, default=0.42)
    parser.add_argument("--sprite-drop-retreat-distance", type=float, default=0.42)
    parser.add_argument("--post-turn-retreat-distance", type=float, default=0.08)
    parser.add_argument("--search-forward-distance", type=float, default=0.10)
    parser.add_argument("--search-move-speed", type=float, default=0.6)
    parser.add_argument("--max-search-forward-steps", type=int, default=10)
    parser.add_argument(
        "--calibration-output",
        type=Path,
        default=Path("robomaster_strafe_calibration.json"),
    )
    return parser.parse_args()


def connect_with_retry(host, port, timeout):
    deadline = time.time() + timeout
    last_error = None
    while time.time() < deadline:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3.0)
        try:
            sock.connect((host, port))
            return sock
        except OSError as exc:
            last_error = exc
            sock.close()
            time.sleep(0.5)
    raise ConnectionError("无法连接视觉进程：%s" % last_error)


def append_safety_log(path, message):
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as stream:
            stream.write(
                "%s | %s\n"
                % (datetime.now().isoformat(timespec="seconds"), message)
            )
    except Exception:
        pass


def stop_chassis(chassis):
    if chassis is not None:
        try:
            chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0, timeout=0.3)
        finally:
            # Also cancel any position Action left by an earlier SDK program.
            chassis.stop()


def run_wheel_pulse(
    chassis, wheel_speeds, duration, soft_stop_rpm=0, soft_stop_duration=0.0
):
    w1, w2, w3, w4 = wheel_speeds
    ramp_duration = min(max(soft_stop_duration, 0.0), duration)
    cruise_duration = duration - ramp_duration
    try:
        chassis.drive_wheels(
            w1=w1,
            w2=w2,
            w3=w3,
            w4=w4,
            timeout=duration + 0.2,
        )
        if cruise_duration:
            time.sleep(cruise_duration)
        if ramp_duration:
            peak = max(abs(item) for item in wheel_speeds)
            ratio = min(float(abs(soft_stop_rpm)) / peak, 1.0)
            slow_wheels = tuple(int(round(item * ratio)) for item in wheel_speeds)
            chassis.drive_wheels(
                w1=slow_wheels[0],
                w2=slow_wheels[1],
                w3=slow_wheels[2],
                w4=slow_wheels[3],
                timeout=ramp_duration + 0.2,
            )
            time.sleep(ramp_duration)
    finally:
        stop_chassis(chassis)


def wheel_patterns(rpm, yaw_trim_rpm=0):
    """Return the two pure-strafe Mecanum patterns in SDK wheel order.

    SDK order: w1 right-front, w2 left-front, w3 left-rear,
    w4 right-rear. Positive means that individual wheel drives forward.
    """
    value = abs(int(rpm))
    trim = abs(int(yaw_trim_rpm))
    # The untrimmed left pattern is (+R,-R,+R,-R). On this robot it
    # produces slight counter-clockwise yaw, so mix in a small clockwise
    # component (-T,+T,+T,-T). The right pattern is its exact inverse.
    left = (value - trim, -value + trim, value + trim, -value - trim)
    return {
        "A": left,
        "D": tuple(-item for item in left),
    }


def forward_wheel_pattern(rpm):
    value = abs(int(rpm))
    return (value, value, value, value)


def combined_approach_pattern(forward_rpm, lateral_rpm):
    """Mix forward motion with lateral correction; positive lateral is right."""
    forward = abs(int(forward_rpm))
    lateral = int(round(lateral_rpm))
    return (
        forward - lateral,
        forward + lateral,
        forward - lateral,
        forward + lateral,
    )


def shortest_angle_error(target_deg, current_deg):
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def correct_heading(chassis, attitude_state, reference_yaw, args):
    """Restore the yaw captured when auto alignment was armed."""
    if reference_yaw is None or attitude_state["yaw"] is None:
        return False
    if time.monotonic() - attitude_state["updated"] > 1.0:
        print("姿态数据已过期，跳过本次航向修正。")
        return False
    error = shortest_angle_error(reference_yaw, attitude_state["yaw"])
    if abs(error) <= args.heading_tolerance_deg:
        return False
    correction = max(
        -args.heading_max_correction_deg,
        min(error, args.heading_max_correction_deg),
    )
    print(
        "航向保持：当前=%+.2f°, 参考=%+.2f°, 修正=%+.2f°"
        % (attitude_state["yaw"], reference_yaw, correction)
    )
    action = chassis.move(x=0, y=0, z=correction, z_speed=args.heading_correction_speed)
    completed = action.wait_for_completed(timeout=2.0)
    stop_chassis(chassis)
    if not completed:
        print("航向修正超时，已发送停车指令。")
    return completed


def move_arm_to(arm, x_mm, y_mm, timeout, label):
    print("机械臂移动到 (%d, %d) mm：%s" % (x_mm, y_mm, label))
    action = arm.moveto(x=x_mm, y=y_mm)
    if not action.wait_for_completed(timeout=timeout):
        arm.stop()
        raise RuntimeError("机械臂移动超时：%s" % label)


def prepare_grasp_pose(chassis, arm, gripper, args):
    stop_chassis(chassis)
    move_arm_to(
        arm,
        args.arm_x_mm,
        args.arm_ready_y_mm,
        args.arm_action_timeout,
        "启动时视觉避让准备位",
    )
    print("张开夹爪，等待 %.1f 秒。" % args.gripper_wait)
    gripper.open()
    time.sleep(args.gripper_wait)


def grasp_and_lift(chassis, arm, gripper, args):
    stop_chassis(chassis)
    if args.pre_grasp_settle:
        print("抓取前等待底盘静止 %.1f 秒。" % args.pre_grasp_settle)
        time.sleep(args.pre_grasp_settle)
        stop_chassis(chassis)
    move_arm_to(
        arm,
        args.arm_x_mm,
        args.arm_lower_y_mm,
        args.arm_action_timeout,
        "停车后下降到抓取位",
    )
    print("底盘已停止，闭合夹爪。")
    gripper.close()
    time.sleep(args.gripper_wait)
    move_arm_to(
        arm,
        args.arm_x_mm,
        args.arm_lift_y_mm,
        args.arm_action_timeout,
        "抓取后抬升",
    )
    print("抓取并抬升完成。")


def run_position_move(chassis, x, y, speed, timeout, label):
    """Run one guarded relative chassis action and always finish at zero speed."""
    print("%s：x=%+.3f m, y=%+.3f m" % (label, x, y))
    action = chassis.move(x=x, y=y, z=0, xy_speed=speed)
    completed = action.wait_for_completed(timeout=timeout)
    stop_chassis(chassis)
    if not completed:
        raise RuntimeError("底盘位置动作超时：%s" % label)
    time.sleep(0.3)


def require_fresh_position(position_state, max_age=2.0):
    value = position_state.get("value")
    if value is None or time.monotonic() - position_state.get("updated", 0.0) > max_age:
        raise RuntimeError("底盘位置数据不可用，拒绝执行返回/放置动作")
    return value


def return_to_mission_origin(chassis, position_state, origin, args):
    """Back away from the grid first, then remove residual lateral error."""
    current = require_fresh_position(position_state)
    dx = origin[0] - current[0]
    if abs(dx) >= 0.02:
        run_position_move(
            chassis, dx, 0.0, args.sort_move_speed,
            args.sort_action_timeout, "后退到初始横线",
        )
    current = require_fresh_position(position_state)
    dy = origin[1] - current[1]
    if abs(dy) >= 0.02:
        run_position_move(
            chassis, 0.0, dy, args.sort_move_speed,
            args.sort_action_timeout, "横移回初始中线",
        )


def retreat_to_initial_line(chassis, position_state, origin, args):
    """Retreat only along x; preserve the picked cell's lateral position."""
    current = require_fresh_position(position_state)
    dx = origin[0] - current[0]
    if abs(dx) >= 0.02:
        run_position_move(
            chassis, dx, 0.0, args.sort_move_speed,
            args.sort_action_timeout, "抓取后垂直后退到初始横线",
        )


def run_turn(chassis, degrees, args, label):
    print("%s：%+.1f°。" % (label, degrees))
    action = chassis.move(x=0, y=0, z=degrees, z_speed=30)
    if not action.wait_for_completed(timeout=args.sort_action_timeout):
        stop_chassis(chassis)
        raise RuntimeError("%s超时" % label)
    stop_chassis(chassis)
    time.sleep(0.5)


def fixed_side_drop_and_continue(
    chassis, arm, gripper, position_state, origin, picked_label, args
):
    retreat_to_initial_line(chassis, position_state, origin, args)
    if picked_label == "coke_can":
        outward_turn = abs(args.fixed_left_turn_deg)
        return_turn = -abs(args.fixed_return_turn_deg)
        forward_distance = abs(args.fixed_drop_forward_distance)
        retreat_distance = abs(args.fixed_drop_retreat_distance)
        side_name = "可乐左侧"
    elif picked_label == "sprite_can":
        outward_turn = -abs(args.fixed_left_turn_deg)
        return_turn = abs(args.fixed_return_turn_deg)
        forward_distance = abs(args.sprite_drop_forward_distance)
        retreat_distance = abs(args.sprite_drop_retreat_distance)
        side_name = "雪碧右侧"
    else:
        raise RuntimeError("固定流程不支持类别：%s" % picked_label)

    run_turn(chassis, outward_turn, args, "%s转向放置侧" % side_name)
    run_position_move(
        chassis, forward_distance, 0.0, args.sort_move_speed,
        args.sort_action_timeout, "%s前进到放置点" % side_name,
    )
    move_arm_to(
        arm, args.arm_x_mm, args.arm_lower_y_mm,
        args.arm_action_timeout, "%s下降放置" % side_name,
    )
    print("%s张开夹爪。" % side_name)
    gripper.open()
    time.sleep(args.gripper_wait)
    move_arm_to(
        arm, args.arm_x_mm, args.arm_ready_y_mm,
        args.arm_action_timeout, "放置后抬到避让位",
    )
    run_position_move(
        chassis, -retreat_distance, 0.0, args.sort_move_speed,
        args.sort_action_timeout, "%s放置后后退" % side_name,
    )
    run_turn(chassis, return_turn, args, "%s转回六宫格方向" % side_name)
    run_position_move(
        chassis, -abs(args.post_turn_retreat_distance), 0.0,
        args.sort_move_speed, args.sort_action_timeout,
        "%s转回后再次后退" % side_name,
    )
    print("%s放置流程完成，继续下一轮识别。" % side_name)


def place_to_side_and_return(chassis, arm, gripper, position_state, origin, label, args):
    return_to_mission_origin(chassis, position_state, origin, args)
    if label == "coke_can":
        side_y = abs(args.drop_lateral_distance)
        side_name = "左侧（可乐）"
    elif label == "sprite_can":
        side_y = -abs(args.drop_lateral_distance)
        side_name = "右侧（雪碧）"
    else:
        raise RuntimeError("未知目标类别，拒绝放置：%s" % label)
    run_position_move(
        chassis, 0.0, side_y, args.sort_move_speed,
        args.sort_action_timeout, "携带罐子前往%s" % side_name,
    )
    move_arm_to(
        arm, args.arm_x_mm, args.arm_lower_y_mm,
        args.arm_action_timeout, "下降到放置位",
    )
    print("在%s张开夹爪。" % side_name)
    gripper.open()
    time.sleep(args.gripper_wait)
    move_arm_to(
        arm, args.arm_x_mm, args.arm_ready_y_mm,
        args.arm_action_timeout, "放置后回到视觉避让位",
    )
    run_position_move(
        chassis, 0.0, -side_y, args.sort_move_speed,
        args.sort_action_timeout, "返回初始中线",
    )
    return_to_mission_origin(chassis, position_state, origin, args)


def save_calibration(args, pulse_counts):
    print("\n横移测试已停车。请根据实际观察记录方向。")
    if pulse_counts["A"] < 1 or pulse_counts["D"] < 1:
        print(
            "A、D 必须各执行至少一次；本次计数 A={}，D={}，不保存结果。".format(
                pulse_counts["A"], pulse_counts["D"]
            )
        )
        return
    while True:
        answer = input("哪个键让底盘向物理左侧移动？请输入 A、D 或 SKIP：").strip().upper()
        if answer in ("A", "D", "SKIP"):
            break
        print("输入无效。")
    if answer == "SKIP":
        print("本次未保存方向标定结果。")
        return

    patterns = wheel_patterns(args.wheel_rpm, args.yaw_trim_rpm)
    left_pattern = patterns[answer]
    right_key = "D" if answer == "A" else "A"
    right_pattern = patterns[right_key]
    result = {
        "timestamp": datetime.now().isoformat(),
        "mode": "strafe-calibration",
        "wheel_rpm": abs(args.wheel_rpm),
        "yaw_trim_rpm": abs(args.yaw_trim_rpm),
        "soft_stop_rpm": abs(args.soft_stop_rpm),
        "soft_stop_duration_s": args.soft_stop_duration,
        "pulse_duration_s": args.pulse_duration,
        "sdk_wheel_order": ["right_front", "left_front", "left_rear", "right_rear"],
        "a_wheels_rpm": list(patterns["A"]),
        "d_wheels_rpm": list(patterns["D"]),
        "physical_left_key": answer,
        "physical_left_wheels_rpm": list(left_pattern),
        "physical_right_key": right_key,
        "physical_right_wheels_rpm": list(right_pattern),
        "executed_pulses": dict(pulse_counts),
    }
    args.calibration_output.parent.mkdir(parents=True, exist_ok=True)
    with args.calibration_output.open("w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print("标定结果已保存：%s" % args.calibration_output.resolve())


def main():
    args = parse_args()
    if not 1 <= abs(args.wheel_rpm) <= 1000:
        raise ValueError("--wheel-rpm 的绝对值必须在 1 到 1000 之间")
    if not 0 <= abs(args.yaw_trim_rpm) < abs(args.wheel_rpm):
        raise ValueError("--yaw-trim-rpm 必须不小于 0 且小于 --wheel-rpm")
    if not 0.05 <= args.pulse_duration <= 1.0:
        raise ValueError("--pulse-duration 必须在 0.05 到 1.0 秒之间")
    if not 0 <= abs(args.soft_stop_rpm) < abs(args.wheel_rpm):
        raise ValueError("--soft-stop-rpm 必须不小于 0 且小于 --wheel-rpm")
    if not 0 <= args.soft_stop_duration < args.pulse_duration:
        raise ValueError("--soft-stop-duration 必须不小于 0 且小于 --pulse-duration")
    if args.max_auto_pulses < 1:
        raise ValueError("--max-auto-pulses 必须至少为 1")
    if not 1 <= abs(args.forward_wheel_rpm) <= 1000:
        raise ValueError("--forward-wheel-rpm 的绝对值必须在 1 到 1000 之间")
    if not 1 <= abs(args.max_steer_rpm) < abs(args.forward_wheel_rpm):
        raise ValueError("--max-steer-rpm 必须至少为 1 且小于 --forward-wheel-rpm")
    if args.max_forward_pulses < 1:
        raise ValueError("--max-forward-pulses 必须至少为 1")
    if not 0 <= args.heading_tolerance_deg < args.heading_max_correction_deg:
        raise ValueError("航向容差必须不小于 0 且小于最大修正角")
    if not 0 < args.heading_max_correction_deg <= 10:
        raise ValueError("最大航向修正角必须在 0 到 10 度之间")
    if not 10 <= args.heading_correction_speed <= 540:
        raise ValueError("航向修正速度必须在 10 到 540 度/秒之间")
    if not 0 <= args.heading_settle_time <= 1.0:
        raise ValueError("航向稳定等待时间必须在 0 到 1 秒之间")
    if args.grasp_after_approach and args.control_mode != "auto-approach":
        raise ValueError("--grasp-after-approach 只能用于 auto-approach 模式")
    if args.sort_mission and not args.grasp_after_approach:
        raise ValueError("--sort-mission 必须与 --grasp-after-approach 一起使用")
    if args.first_left_drop_test and not args.sort_mission:
        raise ValueError("--first-left-drop-test 必须与 --sort-mission 一起使用")
    if not 1 <= abs(args.fixed_left_turn_deg) <= 180:
        raise ValueError("固定左转角度必须在 1 到 180 度之间")
    if not 1 <= abs(args.fixed_return_turn_deg) <= 180:
        raise ValueError("固定返回右转角度必须在 1 到 180 度之间")
    if not 0.05 <= abs(args.fixed_drop_forward_distance) <= 1.0:
        raise ValueError("固定放置前进距离必须在 0.05 到 1.0 米之间")
    if not 0.05 <= abs(args.fixed_drop_retreat_distance) <= 1.0:
        raise ValueError("固定放置后退距离必须在 0.05 到 1.0 米之间")
    if not 0.05 <= abs(args.sprite_drop_forward_distance) <= 1.0:
        raise ValueError("雪碧放置前进距离必须在 0.05 到 1.0 米之间")
    if not 0.05 <= abs(args.sprite_drop_retreat_distance) <= 1.0:
        raise ValueError("雪碧放置后退距离必须在 0.05 到 1.0 米之间")
    if not 0.05 <= abs(args.post_turn_retreat_distance) <= 1.0:
        raise ValueError("转回后再次后退距离必须在 0.05 到 1.0 米之间")
    if not 0.02 <= args.search_forward_distance <= 0.30:
        raise ValueError("搜索前进距离必须在 0.02 到 0.30 米之间")
    if not 0.1 <= args.search_move_speed <= 2.0:
        raise ValueError("搜索移动速度必须在 0.1 到 2.0 m/s 之间")
    if not 1 <= args.max_search_forward_steps <= 30:
        raise ValueError("搜索前进次数上限必须在 1 到 30 之间")
    if not 0.26 <= args.drop_lateral_distance <= 1.0:
        raise ValueError("左右放置距离必须在 0.26 到 1.0 米之间")
    if not 0.5 <= args.sort_move_speed <= 2.0:
        raise ValueError("分拣移动速度必须在 0.5 到 2.0 m/s 之间")
    if args.sort_action_timeout < 3.0:
        raise ValueError("分拣位置动作超时必须至少为 3 秒")
    if not 1 <= args.max_missing_camera_frames <= 30:
        raise ValueError("连续摄像头空帧上限必须在 1 到 30 之间")
    if not 3 <= args.max_identical_camera_frames <= 60:
        raise ValueError("连续相同摄像头帧上限必须在 3 到 60 之间")
    if not 0.5 <= args.camera_freeze_timeout <= 10.0:
        raise ValueError("摄像头冻结超时必须在 0.5 到 10 秒之间")
    if (
        args.arm_action_timeout <= 0
        or args.gripper_wait < 0
        or not 0 <= args.pre_grasp_settle <= 3.0
    ):
        raise ValueError("机械臂超时必须大于 0，夹爪等待时间不得小于 0")
    ep_robot = None
    stream_started = False
    sock = None
    chassis = None
    arm = None
    gripper = None
    calibration_session_started = False
    pulse_counts = {"A": 0, "D": 0}
    forward_pulse_count = 0
    search_forward_step_count = 0
    control_was_unlocked = False
    motion_command_sent = False
    attitude_subscribed = False
    position_subscribed = False
    attitude_state = {"yaw": None, "updated": 0.0}
    position_state = {"value": None, "updated": 0.0}
    mission_origin = None
    heading_reference_yaw = None
    grasp_executed = False

    def attitude_callback(attitude_info):
        yaw, _pitch, _roll = attitude_info
        attitude_state["yaw"] = float(yaw)
        attitude_state["updated"] = time.monotonic()

    def position_callback(position_info):
        x, y, z = position_info
        position_state["value"] = (float(x), float(y), float(z))
        position_state["updated"] = time.monotonic()

    try:
        import cv2
        from robomaster import camera, config, conn, robot

        if args.robot_ip:
            config.ROBOT_IP_STR = args.robot_ip

        print("等待本机 YOLO 视觉进程...")
        sock = connect_with_retry(args.host, args.port, args.connect_timeout)
        sock.settimeout(5.0)

        print("连接 RoboMaster EP（%s）..." % args.conn_type)
        ep_robot = robot.Robot()
        # SDK 0.1.1.68 在 conn.py 中错误地用 `is` 比较字符串。
        # argparse 产生的 "ap"/"sta" 可能不是同一个字符串对象，继而造成
        # UnboundLocalError: proxy_addr referenced before assignment。
        sdk_conn_type = (
            conn.CONNECTION_WIFI_AP
            if args.conn_type == "ap"
            else conn.CONNECTION_WIFI_STA
        )
        ep_robot.initialize(conn_type=sdk_conn_type)
        if args.control_mode in ("strafe-calibration", "auto-align", "auto-approach"):
            # GIMBAL_LEAD makes the chassis rotate to follow an off-center
            # gimbal without any drive_speed command. FREE explicitly decouples
            # the chassis and gimbal immediately after initialization.
            if ep_robot.set_robot_mode(mode=robot.FREE) is False:
                raise RuntimeError("无法将机器人切换到 FREE 模式")
            chassis = ep_robot.chassis
            print(
                "机器人模式：%s；按 S 启用控制前不会发送底盘速度指令。"
                % ep_robot.get_robot_mode()
            )
            if args.control_mode == "strafe-calibration":
                calibration_session_started = True
                print(
                    "横移标定模式：S 解锁；A/D 单次横移；X 停车锁定；Q/Esc 停车退出。"
                )
            else:
                if chassis.sub_attitude(freq=20, callback=attitude_callback) is False:
                    raise RuntimeError("无法订阅底盘姿态数据")
                attitude_subscribed = True
                if args.sort_mission:
                    if chassis.sub_position(cs=1, freq=20, callback=position_callback) is False:
                        raise RuntimeError("无法订阅底盘位置数据")
                    position_subscribed = True
                if args.control_mode == "auto-align":
                    print(
                        "自动对齐模式：S 记录当前航向并开始；横移后自动修正偏航。"
                    )
                else:
                    print(
                        "自动接近模式：对中后小步前进，每步重新识别并修正横向与航向。"
                    )
                    if args.grasp_after_approach:
                        arm = ep_robot.robotic_arm
                        gripper = ep_robot.gripper
                        prepare_grasp_pose(chassis, arm, gripper, args)
                        print("机械臂和夹爪已就绪；到达后将自动抓取并抬升。")
                        if args.sort_mission:
                            deadline = time.time() + 3.0
                            while position_state["value"] is None and time.time() < deadline:
                                time.sleep(0.05)
                            mission_origin = require_fresh_position(position_state)
                            print(
                                "六宫格任务初始坐标：x=%+.3f, y=%+.3f, z=%+.2f"
                                % mission_origin
                            )
        ep_camera = ep_robot.camera
        stream_result = ep_camera.start_video_stream(
            display=False, resolution=camera.STREAM_720P
        )
        if stream_result is False:
            raise RuntimeError("RoboMaster 摄像头视频流启动失败")
        stream_started = True
        if args.control_mode == "preview":
            print("摄像头已开启；预览模式不会控制底盘、机械臂或夹爪。")
        elif args.grasp_after_approach:
            print("摄像头已开启；自动接近完成后将执行一次抓取和抬升。")
        else:
            print("摄像头已开启；仅允许受保护的横移脉冲，不控制机械臂或夹爪。")

        frame_id = 0
        cycle_id = 0
        last_grasp_cycle_id = -1
        missing_camera_frames = 0
        identical_camera_frames = 0
        last_frame_crc = None
        last_frame_change_time = time.monotonic()
        while True:
            frame = ep_camera.read_cv2_image(timeout=1.0, strategy="newest")
            if frame is None:
                missing_camera_frames += 1
                stop_chassis(chassis)
                if missing_camera_frames >= args.max_missing_camera_frames:
                    raise RuntimeError(
                        "摄像头连续 %d 帧无数据，已锁定运动和抓取"
                        % missing_camera_frames
                    )
                continue
            missing_camera_frames = 0
            ok, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), args.jpeg_quality]
            )
            if not ok:
                stop_chassis(chassis)
                raise RuntimeError("摄像头画面编码失败，已锁定运动和抓取")
            encoded_bytes = encoded.tobytes()
            frame_crc = zlib.crc32(encoded_bytes) & 0xFFFFFFFF
            if frame_crc == last_frame_crc:
                identical_camera_frames += 1
            else:
                identical_camera_frames = 1
                last_frame_crc = frame_crc
                last_frame_change_time = time.monotonic()
            frozen_seconds = time.monotonic() - last_frame_change_time
            if (
                identical_camera_frames >= args.max_identical_camera_frames
                and frozen_seconds >= args.camera_freeze_timeout
            ):
                stop_chassis(chassis)
                raise RuntimeError(
                    "摄像头连续 %d 帧、持续 %.2f 秒完全相同，疑似画面冻结；已禁止抓取"
                    % (identical_camera_frames, frozen_seconds)
                )
            frame_id += 1
            send_packet(
                sock,
                {
                    "type": "frame",
                    "frame_id": frame_id,
                    "cycle_id": cycle_id,
                    "width": int(frame.shape[1]),
                    "height": int(frame.shape[0]),
                },
                encoded_bytes,
            )
            response, _ = recv_packet(sock)
            if int(response.get("frame_id", -1)) != frame_id:
                stop_chassis(chassis)
                raise RuntimeError("视觉响应帧号不匹配，拒绝执行控制命令")
            if int(response.get("cycle_id", -1)) != cycle_id:
                stop_chassis(chassis)
                raise RuntimeError("视觉响应轮次不匹配，拒绝执行控制命令")
            command = response.get("command")
            control_unlocked = bool(response.get("control_unlocked", False))
            if command == "quit":
                stop_chassis(chassis)
                print("视觉窗口请求退出。")
                break
            if command == "mission_complete":
                stop_chassis(chassis)
                print("六宫格中已无目标或已完成 6 次分拣，任务结束。")
                break
            if grasp_executed:
                # A completed pick is terminal for this run. Ignore a later S
                # key or any stale motion command while the object is held.
                continue
            if args.control_mode == "preview":
                # Defense in depth: preview mode ignores every motion command.
                continue
            if not control_was_unlocked and control_unlocked:
                search_forward_step_count = 0
                heading_reference_yaw = attitude_state["yaw"]
                if heading_reference_yaw is None:
                    print("等待首个姿态数据，收到后将设为航向参考。")
                else:
                    print("已记录航向参考：%+.2f°" % heading_reference_yaw)
            if control_was_unlocked and not control_unlocked:
                stop_chassis(chassis)
                heading_reference_yaw = None
            control_was_unlocked = control_unlocked
            # After any motion has occurred, keep asserting zero wheel speed
            # on every locked frame. This prevents a delayed/stale SDK wheel
            # command from letting the chassis creep after a safety lock.
            if not control_unlocked and motion_command_sent and command == "continue":
                stop_chassis(chassis)
            if command == "stop":
                print("收到 X/锁定命令：底盘停车。")
                stop_chassis(chassis)
                if not bool(response.get("awaiting_cycle_rearm", False)):
                    search_forward_step_count = 0
            elif command == "aligned":
                print("目标已进入中轴容差带：底盘停车并锁定。")
                stop_chassis(chassis)
            elif command == "approach_complete":
                print("目标已对中并达到视觉停止距离：底盘停车并锁定。")
                stop_chassis(chassis)
                if args.grasp_after_approach:
                    if last_grasp_cycle_id == cycle_id:
                        raise RuntimeError("同一轮收到重复抓取命令，已拒绝执行")
                    target = response.get("target") or {}
                    if not target.get("label"):
                        raise RuntimeError("抓取命令缺少目标信息，已拒绝执行")
                    last_grasp_cycle_id = cycle_id
                    grasp_and_lift(chassis, arm, gripper, args)
                    if args.sort_mission:
                        picked_label = str(target.get("label", "")).lower()
                        print("本轮抓取类别：%s" % picked_label)
                        if args.first_left_drop_test:
                            fixed_side_drop_and_continue(
                                chassis,
                                arm,
                                gripper,
                                position_state,
                                mission_origin,
                                picked_label,
                                args,
                            )
                        else:
                            place_to_side_and_return(
                                chassis,
                                arm,
                                gripper,
                                position_state,
                                mission_origin,
                                picked_label,
                                args,
                            )
                        stop_chassis(chassis)
                        send_packet(
                            sock,
                            {
                                "type": "cycle_complete",
                                "label": picked_label,
                                "cycle_id": cycle_id,
                            },
                        )
                        cycle_response, _ = recv_packet(sock)
                        next_cycle_id = int(cycle_response.get("cycle_id", -1))
                        if next_cycle_id != cycle_id + 1:
                            raise RuntimeError("分拣轮次确认不匹配，已停止后续抓取")
                        cycle_id = next_cycle_id
                        cycle_command = cycle_response.get("command")
                        completed_count = int(cycle_response.get("sorted_count", 0))
                        print("已完成 %d 个罐子的分拣。" % completed_count)
                        pulse_counts = {"A": 0, "D": 0}
                        forward_pulse_count = 0
                        search_forward_step_count = 0
                        if cycle_command == "mission_complete":
                            print("已达到固定分拣任务上限，任务结束。" if args.first_left_drop_test else "已达到六宫格任务上限，任务结束。")
                            break
                        control_was_unlocked = bool(
                            cycle_response.get("control_unlocked", False)
                        )
                        heading_reference_yaw = attitude_state["yaw"]
                    else:
                        grasp_executed = True
            elif command == "approach_blocked":
                print("目标已很近但未可靠对中：底盘停车，不执行抓取。")
                stop_chassis(chassis)
            elif command == "search_forward_step":
                if args.control_mode != "auto-approach" or not args.sort_mission:
                    stop_chassis(chassis)
                    continue
                if search_forward_step_count >= args.max_search_forward_steps:
                    print("达到客户端搜索前进次数上限，底盘停车等待目标。")
                    stop_chassis(chassis)
                    continue
                if heading_reference_yaw is None:
                    heading_reference_yaw = attitude_state["yaw"]
                requested_distance = response.get("search_distance")
                search_distance = args.search_forward_distance
                if requested_distance is not None:
                    search_distance = min(
                        search_distance,
                        max(0.02, float(requested_distance)),
                    )
                print(
                    "未发现目标，向前移动 %.2f m（%d/%d），停车后重新识别。"
                    % (
                        search_distance,
                        search_forward_step_count + 1,
                        args.max_search_forward_steps,
                    )
                )
                run_position_move(
                    chassis,
                    search_distance,
                    0.0,
                    args.search_move_speed,
                    args.sort_action_timeout,
                    "目标丢失搜索前进",
                )
                motion_command_sent = True
                search_forward_step_count += 1
                if args.heading_settle_time:
                    time.sleep(args.heading_settle_time)
                correct_heading(
                    chassis, attitude_state, heading_reference_yaw, args
                )
            elif command == "pulse_negative_y":
                if args.control_mode != "strafe-calibration":
                    stop_chassis(chassis)
                    continue
                if not control_unlocked:
                    stop_chassis(chassis)
                    continue
                wheels = wheel_patterns(args.wheel_rpm, args.yaw_trim_rpm)["A"]
                print(
                    "执行 A 麦轮脉冲：w1={}, w2={}, w3={}, w4={}, duration={:.2f}s".format(
                        wheels[0], wheels[1], wheels[2], wheels[3], args.pulse_duration
                    )
                )
                run_wheel_pulse(
                    chassis,
                    wheels,
                    args.pulse_duration,
                    args.soft_stop_rpm,
                    args.soft_stop_duration,
                )
                motion_command_sent = True
                pulse_counts["A"] += 1
            elif command == "pulse_positive_y":
                if args.control_mode != "strafe-calibration":
                    stop_chassis(chassis)
                    continue
                if not control_unlocked:
                    stop_chassis(chassis)
                    continue
                wheels = wheel_patterns(args.wheel_rpm, args.yaw_trim_rpm)["D"]
                print(
                    "执行 D 麦轮脉冲：w1={}, w2={}, w3={}, w4={}, duration={:.2f}s".format(
                        wheels[0], wheels[1], wheels[2], wheels[3], args.pulse_duration
                    )
                )
                run_wheel_pulse(
                    chassis,
                    wheels,
                    args.pulse_duration,
                    args.soft_stop_rpm,
                    args.soft_stop_duration,
                )
                motion_command_sent = True
                pulse_counts["D"] += 1
            elif command in ("auto_pulse_left", "auto_pulse_right"):
                if args.control_mode not in ("auto-align", "auto-approach") or not control_unlocked:
                    stop_chassis(chassis)
                    continue
                if pulse_counts["A"] + pulse_counts["D"] >= args.max_auto_pulses:
                    print("达到客户端自动横移脉冲上限，底盘停车。")
                    stop_chassis(chassis)
                    continue
                key_name = "A" if command == "auto_pulse_left" else "D"
                requested_wheel_rpm = response.get("wheel_rpm")
                pulse_wheel_rpm = abs(args.wheel_rpm)
                if requested_wheel_rpm is not None:
                    pulse_wheel_rpm = min(
                        pulse_wheel_rpm,
                        max(1, int(round(abs(float(requested_wheel_rpm))))),
                    )
                wheels = wheel_patterns(pulse_wheel_rpm, args.yaw_trim_rpm)[key_name]
                target = response.get("target") or {}
                if heading_reference_yaw is None and attitude_state["yaw"] is not None:
                    heading_reference_yaw = attitude_state["yaw"]
                    print("已记录延迟航向参考：%+.2f°" % heading_reference_yaw)
                error_ratio = float(target.get("error_ratio", 0.0))
                requested_duration = response.get("pulse_duration")
                pulse_duration = min(
                    max(float(requested_duration), 0.05), args.pulse_duration
                ) if requested_duration is not None else args.pulse_duration
                print(
                    "自动横移 %s：dx=%+.3f, w=(%d,%d,%d,%d), %.2fs"
                    % (
                        "左" if key_name == "A" else "右",
                        error_ratio,
                        wheels[0],
                        wheels[1],
                        wheels[2],
                        wheels[3],
                        pulse_duration,
                    )
                )
                run_wheel_pulse(
                    chassis,
                    wheels,
                    pulse_duration,
                    args.soft_stop_rpm,
                    args.soft_stop_duration,
                )
                motion_command_sent = True
                pulse_counts[key_name] += 1
                if args.heading_settle_time:
                    time.sleep(args.heading_settle_time)
                correct_heading(
                    chassis, attitude_state, heading_reference_yaw, args
                )
            elif command == "auto_pulse_forward":
                if args.control_mode != "auto-approach" or not control_unlocked:
                    stop_chassis(chassis)
                    continue
                if forward_pulse_count >= args.max_forward_pulses:
                    print("达到客户端前进脉冲上限，底盘停车。")
                    stop_chassis(chassis)
                    continue
                target = response.get("target") or {}
                requested_duration = response.get("pulse_duration")
                pulse_duration = min(
                    max(float(requested_duration), 0.05), args.pulse_duration
                ) if requested_duration is not None else min(0.25, args.pulse_duration)
                wheels = forward_wheel_pattern(args.forward_wheel_rpm)
                print(
                    "自动前进：height=%.3f, w=(%d,%d,%d,%d), %.2fs"
                    % (
                        float(target.get("height_ratio", 0.0)),
                        wheels[0],
                        wheels[1],
                        wheels[2],
                        wheels[3],
                        pulse_duration,
                    )
                )
                run_wheel_pulse(
                    chassis,
                    wheels,
                    pulse_duration,
                    args.soft_stop_rpm,
                    args.soft_stop_duration,
                )
                motion_command_sent = True
                forward_pulse_count += 1
                if args.heading_settle_time:
                    time.sleep(args.heading_settle_time)
                correct_heading(
                    chassis, attitude_state, heading_reference_yaw, args
                )
            elif command == "auto_drive_approach":
                if args.control_mode != "auto-approach" or not control_unlocked:
                    stop_chassis(chassis)
                    continue
                if forward_pulse_count >= args.max_forward_pulses:
                    print("达到客户端前进脉冲上限，底盘停车。")
                    stop_chassis(chassis)
                    continue
                target = response.get("target") or {}
                requested_duration = response.get("pulse_duration")
                pulse_duration = min(
                    max(float(requested_duration), 0.05), args.pulse_duration
                ) if requested_duration is not None else min(0.25, args.pulse_duration)
                requested_lateral = float(response.get("lateral_rpm") or 0.0)
                lateral_rpm = max(
                    -abs(args.max_steer_rpm),
                    min(requested_lateral, abs(args.max_steer_rpm)),
                )
                wheels = combined_approach_pattern(
                    args.forward_wheel_rpm, lateral_rpm
                )
                print(
                    "PID 合成前进：dx=%+.3f, centerY=%.3f/filtered=%.3f, "
                    "height=%.3f, lateral=%+.1frpm, "
                    "w=(%d,%d,%d,%d), %.2fs"
                    % (
                        float(target.get("error_ratio", 0.0)),
                        float(target.get("center_y_ratio", 0.0)),
                        float(target.get("distance_center_y_ratio", target.get("center_y_ratio", 0.0))),
                        float(target.get("height_ratio", 0.0)),
                        lateral_rpm,
                        wheels[0],
                        wheels[1],
                        wheels[2],
                        wheels[3],
                        pulse_duration,
                    )
                )
                run_wheel_pulse(
                    chassis,
                    wheels,
                    pulse_duration,
                    args.soft_stop_rpm,
                    args.soft_stop_duration,
                )
                motion_command_sent = True
                forward_pulse_count += 1
                if args.heading_settle_time:
                    time.sleep(args.heading_settle_time)
                correct_heading(
                    chassis, attitude_state, heading_reference_yaw, args
                )
        if calibration_session_started:
            save_calibration(args, pulse_counts)
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在关闭摄像头。")
        return 130
    except Exception as exc:
        error_text = "错误：%s: %s" % (type(exc).__name__, exc)
        print(error_text, file=sys.stderr)
        append_safety_log(args.safety_log, error_text)
        return 1
    finally:
        try:
            stop_chassis(chassis)
        except Exception:
            pass
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass
        if ep_robot is not None:
            if attitude_subscribed and chassis is not None:
                try:
                    chassis.unsub_attitude()
                except Exception:
                    pass
            if position_subscribed and chassis is not None:
                try:
                    chassis.unsub_position()
                except Exception:
                    pass
            if stream_started:
                try:
                    ep_robot.camera.stop_video_stream()
                except Exception:
                    pass
            if arm is not None and not grasp_executed:
                try:
                    arm.stop()
                except Exception:
                    pass
            try:
                ep_robot.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
