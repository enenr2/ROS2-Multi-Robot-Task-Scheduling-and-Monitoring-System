# ROS2 多机器人任务调度与监控系统

两台 TurtleBot3 在 Gazebo 中自主作业：**视觉发现彩色目标 → 调度分配任务 → Nav2 导航到目标附近 → 视觉闭环精对位停车**，全过程通过话题与 action 可观测、可回放。

不同于只展示 3D 模型的仿真项目，本仓库的重点在工程细节：多机命名空间隔离、TF 树统一、接口版本化、纯逻辑单测、故障注入与状态可观测。

当前状态：**M0–M5 已完成并通过仿真验收**，M6–M9 开发中。

---

## 1. 系统架构

```
                     Gazebo (gz sim 10)  +  fleet_tasks.sdf 世界
                                  │
                        ros_gz_bridge (单车 8 类话题)
                                  │
      ┌───────────────────────────┼────────────────────────────┐
      ▼                           ▼                            ▼
 /robotN/odom            /robotN/camera/image           /robotN/scan
 /robotN/imu             /robotN/camera/camera_info     /robotN/cmd_vel
      │                           │                            │
      ▼                           ▼                            ▼
 里程计 / TF              fleet_perception              Nav2 (robot1)
                          HSV 分割 + 单目测距            navigate_to_pose
                          + 时序去抖                          │
                              │                               │
                              ▼                               ▼
                     /robotN/detections              action 客户端
                              │                               │
                              └───────────┬───────────────────┘
                                          ▼
                                   fleet_agent
                          ASSIGNED → NAVIGATING → DOCKING
                                          │
                       ┌──────────────────┴─────────────────┐
                       ▼                                    ▼
              /robotN/task_status                  /robotN/robot_status
              （任务状态事件流）                     （1 Hz 心跳，判活用）

  调度层 fleet_manager（M6 接入）── /robotN/execute_task (action) ──▶ fleet_agent
```

任务执行三段式：**导航**（Nav2 负责全局路径与避障，终点退到目标前 1.4 m 并让车头对准目标）→ **对位**（视觉闭环把航向收进 0.05 rad、距离收到 standoff 0.6 m）→ **收尾**（停车、发终态）。

---

## 2. 功能模块

| 包 | 职责 | 状态 | 测试 |
|---|---|---|---|
| `fleet_msgs` | 接口层：4 个 msg、2 个 srv、1 个 action | 完成 | — |
| `fleet_bringup` | 仿真底座：自建世界/地图/机器人模型、桥接配置、RViz、一键 launch | 完成 | — |
| `fleet_manager` | 调度核心：状态机、任务队列、分配策略、看门狗（学员纯逻辑，不依赖 ROS） | 核心完成 | 54 gtest |
| `fleet_perception` | 感知：HSV 颜色识别、单目测距、时序去抖、TF 转世界系 | 完成 | 31 gtest |
| `fleet_agent` | 执行代理：封装 Nav2、视觉精对位、状态与心跳 | 完成 | 17 gtest |

分层原则：**能脱离 ROS 的数学与状态逻辑一律做成纯 C++ 库**，用注入时间驱动单测；ROS 节点只做接线与消息转换。

### 接口定义

| 类型 | 名称 | 用途 |
|---|---|---|
| msg | `Task` | 任务快照，供监控与回放 |
| msg | `TaskStatus` | 任务状态变更事件（含 `failure_reason`、`revision` 语义） |
| msg | `RobotStatus` | 机器人心跳（1 Hz，含递增序号） |
| msg | `Detection` | 视觉检测结果（map 系坐标、测距、不确定度、`lost` 事件标志） |
| srv | `SubmitTask` / `CancelTask` | 外部注入与取消任务 |
| action | `ExecuteTask` | 调度器每车直调，一次调用 = 一个任务全过程 |

---

## 3. 环境依赖

- Ubuntu 26.04 + ROS 2 Lyrical
- Gazebo Sim 10（`ros_gz_sim` 桥接）
- Nav2 1.5.2（`nav2_controller/planner/behaviors/bt_navigator/map_server`）
- OpenCV 4.10（`cv_bridge`）
- Qt6（M8 监控端，已安装）

软件渲染环境下建议固定 `LIBGL_ALWAYS_SOFTWARE=1`（本项目已固化在 `~/.config/environment.d/`）。

---

## 4. 快速开始

```bash
# 1. 依赖（自动探测 ROS 发行版，幂等，逐项累计失败）
./scripts/install_deps.sh

# 2. 构建
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. 一键启动：双机 + 地图 + 桥 + Nav2 + 感知 + 执行代理 + RViz
ros2 launch fleet_bringup fleet_sim.launch.py gui:=false rviz:=true
```

常用开关：`gui:=false|true`、`rviz:=false|true`、`nav2:=false|true`、`perception:=false|true`、`agent:=false|true`。

逐项验收：

```bash
colcon test                                   # 全部单测
colcon test-result --test-result-base build/fleet_manager

# 感知：测距标定 + 三色识别 + 误检率 + 丢失链路
python3 src/fleet_perception/scripts/verify_perception.py

# 执行：导航 → 对位 → 停车 + 并发拒绝 + 目标丢失
python3 src/fleet_agent/scripts/verify_agent.py
```

> 反复 launch/kill 后若某台车的静态 TF 莫名消失（`/tf_static` 里看不到该车连杆），先清理 `/dev/shm/fastdds_*` 再启动 —— 这是 Fast DDS 共享内存段残留，不是代码问题。

---

## 5. 实测验收数据

**感知层**（320×240 / fx=277.19，物块 0.2 m 立方体）

| 项目 | 结果 |
|---|---|
| 单目测距 1 / 2 / 3 m | 误差 +0.019 / +0.023 / −0.031 m（门槛 0.2 m） |
| 三色识别 | 红/绿/蓝各 25 帧全部正确，零串色 |
| 误检率 | 视野内无目标连续 60 s，上报 0 条 |
| 目标丢失 | 移走已捕获目标后，丢失事件恰好上报 1 次 |
| 上报率 | 约 4.5 Hz，与软件渲染下的相机帧率一致 |
| 偏轴退化 | 12° 偏小 0.05 m；24° 偏小 0.17~0.20 m（轮廓外接框把侧棱投影算进宽度） |

**执行层**

| 项目 | 结果 |
|---|---|
| 3 m 外目标全流程 | 导航 + 对位 + 停车，耗时 17.4 s，`SUCCEEDED` |
| 停车距离 | 0.693 m（目标 0.6 m，门槛 ±0.15 m） |
| 航向误差 | 0.00°（容差 2.9°） |
| 状态序列 | `ASSIGNED → NAVIGATING → DOCKING`，`task_status` 终态 `DONE`、progress 1.0 |
| 心跳 | 执行期 `BUSY`、结束回 `IDLE`、序号递增 |
| 并发下发 | 第二个任务被立即拒绝 |
| 对位中目标丢失 | 进入非成功终态，`failure_reason = TARGET_LOST` |

**导航层**（M2）

| 项目 | 结果 |
|---|---|
| 单机 Nav2 目标点 | 落点误差 0.198 m / 0.2 m，两次均 `SUCCEEDED` |
| 多机隔离 | 双机话题、TF、命名空间无冲突，robot2 零漂移 |

---

## 6. 目录结构

```
fleet_ws/
├── src/
│   ├── fleet_msgs/          # msg / srv / action 接口层
│   ├── fleet_bringup/       # 世界、地图、模型、桥接、Nav2 参数、launch、RViz
│   │   ├── worlds/fleet_tasks.sdf
│   │   ├── maps/            # 与自建世界同源生成的地图
│   │   ├── models/robot1|2/ # 带相机与激光的 TurtleBot3
│   │   ├── config/          # bridge.yaml / nav2_robot1.yaml / fleet.rviz
│   │   └── launch/fleet_sim.launch.py
│   ├── fleet_manager/       # 调度核心（纯逻辑，54 gtest）
│   ├── fleet_perception/    # 颜色 + 测距 + 去抖（31 gtest，含验收脚本）
│   └── fleet_agent/         # 执行代理（17 gtest，含验收脚本）
└── scripts/
    ├── install_deps.sh      # 依赖安装（幂等、逐项累计失败）
    └── commit.sh            # 固化提交身份、过滤协作标记
```

---

## 7. 工程要点与踩坑记录

这些问题都在仿真里实际踩过，修复方式已固化进代码：

**多机器人**
- 不同机器人的 URDF 连杆/关节名必须在生成期加全局前缀 —— 只靠 `frame_prefix` 会导致 RViz 同名帧覆盖、模型轮流显示
- 官方 Nav2 launch 会把 `/tf` 重映射成相对名，在命名空间下变成 `/robotN/tf`；多机必须强制回全局 `/tf`、`/tf_static`
- Nav2 参数必须以完整限定名或 `/**` 匹配，普通节点名匹配不上
- 全局代价地图默认订阅 `/robotN/map`，共享地图要重映射成完全限定名 `/map`

**Nav2 1.5**
- 默认发布 `TwistStamped`，与桥接的 `Twist` 不兼容，需关闭 `enable_stamped_cmd_vel`
- `lifecycle_manager` 的 `service_timeout` 默认 5 s，在本机高负载下不够，会中止 bringup，已放宽到 20 s
- 低性能仿真优先 RPP 控制器而非 MPPI

**感知**
- tf2 禁止在回调里带超时查 TF（数据靠本节点执行器喂入，带超时等于自己等自己）；图像戳常比最新 TF 早/晚几毫秒，失败时退化为"最新位姿"
- 颜色阈值必须限制饱和度下限，否则灰白墙面与地面会被误判

**并发（执行代理）**
- action 服务端回调组必须用 `Reentrant` + 总线互斥标志，否则新任务请求会排在长时间 `execute` 后面、等前一任务结束才被处理
- 终态只由 action result 承载：feedback 走话题、result 走服务响应，无顺序保证

---

## 8. 已知限制

- 相机在软件渲染下实际约 4 Hz，视觉对位限速 0.15 m/s；激光约 2.3 Hz
- 感知每色只上报一个目标（当前场景三色各一块足够），偏轴测距会系统性偏小
- 目前仅 robot1 接入 Nav2，robot2 作为可观测的第二个实例
- 调度层尚未接入 ROS 适配（M6），监控端尚未开发（M8）

---

## 9. 路线图

| 里程碑 | 内容 | 状态 |
|---|---|---|
| M0 | 环境骨架与接口层 | 完成 |
| M1 | 仿真底座（世界/地图/模型/桥接/一键启动） | 完成 |
| M2 | robot1 接入 Nav2 单机导航 | 完成 |
| M3 | 调度核心逻辑与 54 个单测 | 完成 |
| M4 | 感知层（颜色识别 + 单目测距 + 去抖） | 完成 |
| M5 | 执行代理（Nav2 封装 + 视觉精对位 + 心跳） | 完成 |
| M6 | `fleet_manager` ROS 适配：队列/分配/超时重试接 action，RViz Marker | 进行中 |
| M7 | 故障注入：目标丢失、导航失败、机器人离线、任务超时、重新分配 | 计划 |
| M8 | Qt6 监控端：任务列表、机器人在线状态、告警、日志、相机流、rosbag 时间轴检索 | 计划 |
| M9 | 集成测试、rosbag 回放、Docker、CI、完整交付文档 | 计划 |

---

## 作者

赵威（enenr2） — 2027 届计算机科学与技术，方向：机器人上位机 / ROS2 系统集成。
