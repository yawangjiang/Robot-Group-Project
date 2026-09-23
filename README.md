# RoboMaster 机器人集成小组项目

本仓库用于统一管理机器人集成小组的课程实验、代码、报告与演示材料。

## 项目结构

- `projects/experiment-1/`：Experiment 1，目标检测与识别相关工作。
- `projects/lab2/`：Lab 2，RoboMaster EP ROS 2 仿真与抓取放置任务。
- `projects/project-3/`：第三个项目的预留目录，后续提交到此处。

两个已完成项目从原 GitHub 仓库导入，并保留各自的提交历史。原仓库暂时保留，作为兼容入口与备份。

## 协作约定

1. 每个任务从 `main` 创建独立分支。
2. 修改完成后通过 Pull Request 合并。
3. ROS 2 的 `build/`、`install/`、`log/` 等生成目录不得提交。
4. 不提交密钥、令牌、设备凭据或个人环境配置。
5. 大型模型与数据集优先使用 Git LFS、Release 或外部数据存储。
