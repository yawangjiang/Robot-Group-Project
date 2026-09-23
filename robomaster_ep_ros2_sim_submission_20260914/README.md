# RoboMaster EP Core 二自由度机械臂联合仿真

这是面向 Ubuntu 22.04 / ROS 2 Humble / Gazebo Fortress 的可重复搬运工程。模型采用 RoboMaster EP Core 机械臂的二维竖直平面等效机构，ROS 坐标为 `x` 向前、`z` 向上；规划只控制两个主动关节，不宣称可独立控制六维末端姿态。

## 本机环境记录（2026-09-11）

- NVIDIA Jetson Orin NX Engineering Reference Developer Kit，aarch64
- Ubuntu 22.04.5，L4T R36.5.2（系统未安装 `nvidia-jetpack` 元包，不能仅凭 L4T 无歧义记录 JetPack 元包版本）
- ROS 2 Humble；MoveIt 2 2.5.9；ros2_control 2.54.0 / ros2_controllers 2.53.3
- Gazebo Fortress / Ignition Gazebo 6.18.0
- `gz_ros2_control` 0.7.21 来自 `/home/nvidia/gz_ros2_control_ws` overlay
- 内存 15 GiB（约 12 GiB 可用），工作盘约 100 GiB 可用
- 未发现已有 ROS 包工作区；`/home/nvidia/backport-iwlwifi` 是独立用户仓库，未修改

## 构建和启动

每个新终端先执行：

```bash
cd /home/nvidia/robomaster_ep_ws
./scripts/build.sh
source install/setup.bash
```

入口：

```bash
ros2 launch robomaster_ep_description display.launch.py
ros2 launch robomaster_ep_moveit_config demo.launch.py
ros2 launch robomaster_ep_gazebo gazebo_moveit.launch.py
ros2 launch robomaster_ep_gazebo gazebo_moveit.launch.py headless:=true
```

联合仿真就绪后执行一次或多次搬运：

```bash
ros2 launch robomaster_ep_demos pick_place.launch.py cycles:=5
```

完整无 GUI 验证（默认连续搬运 2 次，任何步骤失败立即返回非零；每轮还会读取 Gazebo 实体位姿，检查方块最终中心与放置点误差不超过 8 mm）：

```bash
./scripts/test_simulation.sh
```

## 控制链和抓取语义

机械臂路径通过 MoveIt `/move_action` 规划与执行，再由 `FollowJointTrajectory`、`arm_controller` 和 `gz_ros2_control` 驱动 Gazebo。夹爪由独立的轨迹控制器同步驱动两个手指。演示持续检查实际 `/joint_states` 的终点误差和有限数值。

假硬件入口使用 `mock_components/GenericSystem`、同名 ros2_control 控制器与墙钟，因此在本机缺少旧版 `MoveItFakeControllerManager` 插件时仍可执行轨迹。

夹持闭合是真实的关节动作，但物块随动采用 Gazebo `SetEntityPose` 的确定性附着层。纯摩擦抓取会随物理引擎、步长和摩擦锥发生偶发滑落，不适合作为“反复完成”的验收基线；本实现只在到达抓取位并闭合后附着，到达放置位后释放。它不是力学意义上的 5 N 夹持仿真。桌面、方块、手指均有独立原语碰撞体，轨迹避开桌面；附着随动由测得关节角做同一套正运动学计算，避免视觉错位。

## 模型来源与许可证

结构和量级参考 [jeguzzi/robomaster_ros](https://github.com/jeguzzi/robomaster_ros)，核对提交 `c05a39d7f0fa8b3b277aa74826aa92e202efc987` 中的 `arm.urdf.xacro`、`gripper.urdf.xacro`、碰撞网格和惯量。该仓库为 MIT License（Copyright 2021 Jérôme Guzzi）。本工程没有复制其 Mesh；使用低复杂度 box 碰撞与视觉体，因此不存在外部 Mesh 安装或路径失效问题。

官方资料只用于工作空间约 220 mm × 150 mm、载荷约 200–300 g、夹爪约 100 mm 和约 5 N 的能力边界；这些宣传规格没有被冒充为精确 CAD 参数。所有近似见 [ASSUMPTIONS.md](ASSUMPTIONS.md)。

## 切换 RoboMaster SDK 实机

保留 MoveIt 和 `arm_controller` 的 FollowJointTrajectory 接口，新增一个 `hardware_interface::SystemInterface`：将肩/肘目标转换成 DJI SDK `robotic_arm.moveto(x, y)`（ROS `x/z` 映射到 SDK `x/y`），将夹爪目标转换成 SDK gripper open/close；周期读取 SDK arm position 和 gripper 状态并发布 state interfaces。启动时把 Xacro 的 `use_gazebo:=false`、`use_fake_hardware:=false`，加载该硬件插件，并加入限速、通信超时、急停和标定偏置。由于官方接口主要是末端增量/位置而非两个电机角度，实机层必须以实测标定表或解析映射闭环，不能直接发送本仿真的等效关节角。

## 已知限制

- 树形二连杆是闭环并联机构的规划等效，不复现所有从动杆动力学。
- 几何、质量、惯量与关节限位中的未公开项是保守近似，不能用于结构强度或力矩选型。
- 确定性附着保证搬运回归稳定，但不验证指尖接触力。
- 相机只提供 TF，不模拟图像传感器；物块位置目前是已知工位。
