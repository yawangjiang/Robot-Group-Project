"""
RoboMaster EP 机械臂定点抓取程序
实验：机械臂定点抓取（真机验证）
物体：网球 | 取放距离：~12cm | 连接方式：WiFi
"""

import time
import json
import logging
import argparse
from datetime import datetime
from robomaster import robot

# ============================================================
#  参数配置（根据实际桌面布局调整）
# ============================================================
ROBOT_IP = "192.168.2.1"          # EP 的 WiFi IP（在 EP 屏幕上查看）

# 机械臂坐标（单位：毫米，原点 = 回中位置）
# X: 前后方向（正值 = 向前伸出）
# Y: 上下方向（正值 = 向上，负值 = 向下）
PICK_X = 180                       # 取物点：手臂向前伸出的距离
PICK_Y = -50                       # 取物点：下降到网球高度
LIFT_Y = 50                        # 抬升高度：抓起后上升多少
PLACE_X = 180                      # 放置点：手臂伸出距离（同取物点）
PLACE_Y = -50                      # 放置点高度

# 底盘旋转
ROTATE_ANGLE = 45                  # 左右各偏转 45°
ROTATE_SPEED = 30                  # 旋转速度 °/s（SDK 范围 [10, 540]）
MOVE_SPEED = 0.5                   # 底盘平移速度（米/秒）

# 夹爪
GRIP_STRENGTH = 3                  # 夹取力度 1-4（网球用 3 档）
GRIP_WAIT = 1.5                    # 夹爪动作等待时间（秒）

# 测试
NUM_TESTS = 5                      # 连续抓取次数


# ============================================================
#  日志
# ============================================================
class Logger:
    def __init__(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = f"pick_place_log_{ts}.json"
        self.trajectory = []
        self.results = []
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
            datefmt="%H:%M:%S",
        )
        self.log = logging.getLogger("EP")

    def info(self, msg):
        self.log.info(msg)

    def error(self, msg):
        self.log.error(msg)

    def record_trajectory(self, action, **kwargs):
        self.trajectory.append({
            "time": datetime.now().isoformat(),
            "action": action,
            **kwargs,
        })

    def save(self):
        data = {
            "timestamp": datetime.now().isoformat(),
            "config": {
                "robot_ip": ROBOT_IP,
                "pick": {"x": PICK_X, "y": PICK_Y},
                "place": {"x": PLACE_X, "y": PLACE_Y},
                "lift_y": LIFT_Y,
                "grip_strength": GRIP_STRENGTH,
            },
            "trajectory": self.trajectory,
            "test_results": self.results,
        }
        with open(self.log_file, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        self.info(f"日志已保存: {self.log_file}")


# ============================================================
#  主控制器
# ============================================================
class PickPlaceController:
    def __init__(self, ip):
        self.logger = Logger()
        self.robot = None
        self.arm = None
        self.gripper = None
        self.chassis = None
        self.gimbal = None
        self.current_heading = 0  # ← 新增：记录当前朝向
        self._connect(ip)

    def rotate_to(self, target_angle, label=""):
        """转到绝对角度"""
        delta = target_angle - self.current_heading
        if abs(delta) < 1:
            return  # 角度没变，不转
        self.logger.info(f"    底盘转到 {target_angle}° (转{delta}°) [{label}]")
        try:
            self.chassis.stop()
        except Exception:
            pass
        time.sleep(0.5)
        self.chassis.move(x=0, y=0, z=delta, z_speed=ROTATE_SPEED)
        time.sleep(4)
        self.current_heading = target_angle  # ← 更新记录
        self.logger.record_trajectory("chassis_rotate_to", angle=target_angle, label=label)

    def _connect(self, ip):
        self.robot = robot.Robot()
        self.robot.initialize(conn_type="ap")

        self.arm = self.robot.robotic_arm
        self.gripper = self.robot.gripper
        self.chassis = self.robot.chassis
        self.gimbal = self.robot.gimbal

        self.chassis.drive_speed = MOVE_SPEED


    # ---------- 基础动作 ----------

    def go_home(self):
        self.logger.info(">>> 回零")
        try:
            self.chassis.stop()
        except Exception:
            pass
        try:
            self.arm.stop()
        except Exception:
            pass
        try:
            self.gimbal.stop()
        except Exception:
            pass
        time.sleep(2)

        self.gripper.open()
        time.sleep(GRIP_WAIT)

        self.arm.moveto(x=0, y=0)
        time.sleep(3)

        self.gimbal.recenter()
        time.sleep(3)

        self.logger.record_trajectory("go_home")
        self.logger.info("    回零完成")

    def move_arm_to(self, x, y, label=""):
        desc = f"({x}, {y})" + (f" [{label}]" if label else "")
        self.logger.info(f"    机械臂 → {desc}")
        self.arm.moveto(x=x, y=y)
        time.sleep(2)
        self.logger.record_trajectory("arm_move", x=x, y=y, label=label)

    def grip_close(self):
        """闭合夹爪"""
        self.logger.info("    夹爪闭合")
        self.gripper.close()
        time.sleep(GRIP_WAIT)
        self.logger.record_trajectory("grip_close")

    def grip_open(self):
        """张开夹爪"""
        self.logger.info("    夹爪张开")
        self.gripper.open()
        time.sleep(GRIP_WAIT)
        self.logger.record_trajectory("grip_open")

    def rotate_chassis(self, angle, label=""):
        """底盘原地旋转，angle: 正值=逆时针(左转), 负值=顺时针(右转)"""
        self.logger.info(f"    底盘旋转 {angle}° [{label}]")
        try:
            self.chassis.stop()
        except Exception:
            pass
        time.sleep(0.5)
        self.chassis.move(x=0, y=0, z=angle, z_speed=ROTATE_SPEED)
        time.sleep(4)
        self.logger.record_trajectory("chassis_rotate", angle=angle, label=label)

    def stop_all(self):
        try:
            self.chassis.stop()
            self.arm.stop()
        except Exception:
            pass
        self.logger.record_trajectory("emergency_stop")

    # ---------- 核心流程 ----------

    def pick(self):
        """取物：移动到取物点 → 下降 → 夹取 → 抬升"""
        self.logger.info(">>> 取物流程")

        # 1. 手臂伸到取物点上方
        self.move_arm_to(PICK_X, LIFT_Y, "取物点上方")

        # 2. 下降到网球高度
        self.move_arm_to(PICK_X, PICK_Y, "下降抓取")

        # 3. 夹取
        self.grip_close()

        # 4. 抬升
        self.move_arm_to(PICK_X, LIFT_Y, "抬升")

        self.logger.info("    取物完成")

    def place(self):
        """放置：移动到放置点 → 下降 → 释放 → 抬升"""
        self.logger.info(">>> 放置流程")

        # 1. 手臂伸到放置点上方
        self.move_arm_to(PLACE_X, LIFT_Y, "放置点上方")

        # 2. 下降到放置高度
        self.move_arm_to(PLACE_X, PLACE_Y, "下降放置")

        # 3. 释放
        self.grip_open()

        # 4. 抬升
        self.move_arm_to(PLACE_X, LIFT_Y, "抬升")

        self.logger.info("    放置完成")

    def pick_and_place_cycle(self):
        self.go_home()
        self.current_heading = 0  # ← 回零后重置朝向
        time.sleep(1)

        count = 0
        try:
            while True:
                count += 1
                self.logger.info(f"\n--- 第 {count} 轮 ---")

                self.rotate_to(-ROTATE_ANGLE, "左前方45°")
                time.sleep(0.5)
                self.pick()
                time.sleep(0.5)

                self.rotate_to(ROTATE_ANGLE, "右前方45°")
                time.sleep(0.5)
                self.place()
                time.sleep(0.5)

                self.pick()  # ← 原地夹取，不转
                time.sleep(0.5)

                self.rotate_to(-ROTATE_ANGLE, "左前方45°")
                time.sleep(0.5)
                self.place()
                time.sleep(0.5)

                self.logger.info(f"    第 {count} 轮完成\n")

        except KeyboardInterrupt:
            self.logger.info("\n用户手动终止")
        except Exception as e:
            self.logger.error(f"出错: {e}")
            self.stop_all()
        finally:
            self.go_home()

    def disconnect(self):
        """断开连接"""
        try:
            self.robot.close()
        except Exception:
            pass


# ============================================================
#  入口
# ============================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RoboMaster EP 定点抓取")
    parser.add_argument("--ip", default=ROBOT_IP, help="EP 的 IP 地址")
    args = parser.parse_args()

    ctrl = PickPlaceController(args.ip)
    try:
        ctrl.logger.info("=" * 50)
        ctrl.logger.info("左右交替抓取循环（Ctrl+C 终止）")
        ctrl.logger.info("=" * 50)

        ctrl.pick_and_place_cycle()

        ctrl.logger.results.append({
            "test_id": 1,
            "success": True,
            "timestamp": datetime.now().isoformat(),
        })
        ctrl.logger.save()

    except Exception as e:
        ctrl.logger.error(f"异常: {e}")
        ctrl.stop_all()
    finally:
        ctrl.disconnect()