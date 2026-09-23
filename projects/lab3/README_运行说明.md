# RoboMaster 六宫格饮料罐分拣（2026-09-15 最新版）

## 运行入口

双击 `run_robomaster_first_left_coke_drop_test.bat`。

运行前：

1. 连接 RoboMaster EP 的 AP Wi-Fi。
2. 确认小车地址为 `192.168.2.1`。
3. 确认 Python 3.7 路径为
   `C:\Users\LENOVO\AppData\Local\Programs\Python\Python37\python.exe`。
4. 确认 YOLO Python 3.8 路径为
   `D:\Miniconda3\envs\yolov8\python.exe`。
5. YOLO 窗口出现后，点击该窗口并按一次 `S` 开始。

`X` 为急停并锁定，`Q` 或 `Esc` 为停车退出。

## 文件说明

- `yolo_six_grid_sort_server_py38.py`：YOLO 识别、PID 决策和任务状态机。
- `robomaster_six_grid_sort_client_py37.py`：摄像头、麦轮、机械臂和夹爪控制。
- `robomaster_split_protocol.py`：两进程之间的 TCP 数据协议。
- `yolo11x_sprite_coke_v1.pt`：可乐/雪碧识别模型。
- `run_robomaster_first_left_coke_drop_test.bat`：完整任务启动入口。
- `README_RoboMaster_六宫格分拣.md`：详细参数和流程说明。

## 当前关键参数

- 目标丢失后连续 10 帧无检测：向前移动 `0.10 m`，停车后重新识别。
- 搜索最多执行 10 次，机械臂在搜索期间不抬高。
- 停滞/兜底抓取下限：`centerY >= 0.515`。
- 主停止线：`0.530`；硬停止线：`0.540`。
- 可乐左侧放置；雪碧右侧放置。

首次运行请清理小车前后和左右通道，并随时准备按 `X` 急停。
