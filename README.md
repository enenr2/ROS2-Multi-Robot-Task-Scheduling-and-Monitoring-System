# ROS2 多机器人任务调度与监控系统

两台 TurtleBot3 在 Gazebo 中自主作业：**视觉发现彩色目标 → 调度分配任务 → Nav2 导航到目标附近 → 视觉闭环精对位停车**，全过程通过话题与 action 可观测、可回放。

不同于只展示 3D 模型的仿真项目，本仓库的重点在工程细节：多机命名空间隔离、TF 树统一、接口版本化、纯逻辑单测、故障注入与状态可观测。

当前状态：**M0–M7 已完成并通过仿真验收**，M8–M9 开发中。

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
 里程计 / TF              fleet_perception          Nav2 (robot1 + robot2)
  + AMCL 定位              HSV 分割 + 单目测距        navigate_to_pose
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

  调度层 fleet_manager ── /robotN/execute_task (action) ──▶ fleet_agent
```

任务执行三段式：**导航**（Nav2 负责全局路径与避障，终点退到目标前 1.4 m 并让车头对准目标）→ **对位**（视觉闭环把航向收进 0.05 rad、距离收到 standoff 0.6 m）→ **收尾**（停车、发终态）。

定位：每台车跑一套 **AMCL**（`/robotN/scan` + 全局 `/map`），由 AMCL 发布 `map → robotN_odom`。gz 的里程计是轮速积分，实测跑十几米会漂 1~3 m；不用 AMCL 时两台车各漂各的，任务改派后新车按旧车报的世界坐标导航会停偏一两米。

---

## 2. 功能模块

| 包 | 职责 | 状态 | 测试 |
|---|---|---|---|
| `fleet_msgs` | 接口层：4 个 msg、2 个 srv、1 个 action | 完成 | — |
| `fleet_bringup` | 仿真底座：自建世界/地图/机器人模型、双 Nav2 + 双 AMCL、桥接配置、RViz、一键 launch | 完成 | — |
| `fleet_manager` | 调度：状态机、任务队列、分配策略、看门狗、失败冷却 + ROS 适配节点（订阅/服务/action/Marker） | 完成 | 54 gtest |
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

常用开关：`gui:=false|true`、`rviz:=false|true`、`nav2:=false|true`、`perception:=false|true`、`agent:=false|true`、`manager:=false|true`。

停仿真用 `./scripts/stop_sim.sh`：会 SIGTERM → 超时 SIGKILL，并清理 `/dev/shm/fastdds_*`。不要用 `pkill -f` 手工杀，卡在生命周期过渡的 Nav2 节点会忽略 SIGTERM 并残留成"同名僵尸节点"，下一次启动时生命周期管理器会卡住。

逐项验收：

```bash
colcon test                                   # 全部单测
colcon test-result --test-result-base build/fleet_manager

# 感知：测距标定 + 三色识别 + 误检率 + 丢失链路
python3 src/fleet_perception/scripts/verify_perception.py

# 执行：导航 → 对位 → 停车 + 并发拒绝 + 目标丢失
python3 src/fleet_agent/scripts/verify_agent.py

# 调度：自动发现 → 分配 → 执行 → 完成 + 服务注入/取消
python3 src/fleet_manager/scripts/verify_manager.py

# 故障注入：四类故障 + 调度层恢复（每个场景建议在干净仿真上单独跑）
python3 src/fleet_manager/scripts/verify_faults.py --scenario A

# 手动制造故障
python3 scripts/fault_injector.py robot-offline --robot robot1 --hold 20
python3 scripts/fault_injector.py nav-failure --robot robot1 --hold 25
python3 scripts/fault_injector.py target-lost --block block_blue
python3 scripts/fault_injector.py target-restore --block block_blue
```

故障场景与预期（详细映射见 `scripts/fault_injector.py` 头部注释）：

| 场景 | 注入手段 | 调度层应看到 | 结果 |
|---|---|---|---|
| A 机器人离线 | `SIGSTOP` 该车 `robot_agent` | 心跳超时 → `ROBOT_UNREACHABLE` → 改派 | 改派后完成 |
| B 任务超时 | 在线把 `nav_timeout` 缩到 2 s | 看门狗 `NAV_TIMEOUT` → 取消 → 改派 | 改派后完成 |
| C 导航栈卡死 | `SIGSTOP` 该车 `controller_server` | `NAV_ABORTED` → 改派（该车 Nav2 整栈下线，不会自愈） | 改派后完成 |
| D 目标丢失 | 对位阶段把物块移出视野 | agent 报 `TARGET_LOST` → 重试一次 | 重试用尽 `FAILED` |

> C 会让故障车的 Nav2 被 lifecycle bond 判死并 deactivate，这是真实的"导航栈需要人工介入"，所以放在最后跑。四个场景建议逐个重启仿真，避免上一轮的里程计/任务残局影响判断。

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

**调度层**（M6）

| 项目 | 结果 |
|---|---|
| 自动发现 | 目标进入视野后自动建单，无需人工注入 |
| 分配 | 派给最近的允许派活车辆（robot1；robot2 无 Nav2 只监控） |
| 状态链 | `PENDING → ASSIGNED → NAVIGATING → DOCKING → DONE` |
| 停车精度 | 车到物块中心 0.82 m（期望约 0.73 m） |
| 监控输出 | `/fleet/tasks` 快照 + `/fleet/task_markers` 持续发布 |
| 服务 | `SubmitTask` 建单、重复提交被去重拒绝、`CancelTask` 中止执行中的任务并回到 IDLE |

**导航层**（M2）

| 项目 | 结果 |
|---|---|
| 单机 Nav2 目标点 | 落点误差 0.198 m / 0.2 m，两次均 `SUCCEEDED` |
| 多机隔离 | 双机话题、TF、命名空间无冲突，robot2 零漂移 |
| AMCL 定位（M7） | `map → base_footprint` 与仿真真值差 0.10 m（此前纯里程计漂移 1~3 m） |

**故障恢复**（M7，每个场景独立干净仿真）

| 场景 | 判定项 | 结果 |
|---|---|---|
| A 机器人离线 | 建单 / 心跳超时重排 / 原因 `ROBOT_UNREACHABLE` / 改派另一台 / 最终 `DONE` | 全通过 |
| B 任务超时 | `nav_timeout` 在线生效 / 看门狗重排 / 原因 `NAV_TIMEOUT` / 改派 / 最终 `DONE` | 全通过 |
| C 导航栈卡死 | 导航失败重排 / 原因 `NAV_ABORTED` / 改派 / 最终 `DONE` | 全通过 |
| D 目标丢失 | 进入对位 / 最终 `FAILED` / 恰好重试 1 次 / 原因 `TARGET_LOST` | 全通过 |

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
│   ├── fleet_manager/       # 调度核心（纯逻辑，54 gtest）+ ROS 适配节点（fleet_manager_node）
│   │   └── scripts/verify_faults.py   # 四类故障注入验收（A/B/C/D）
│   ├── fleet_perception/    # 颜色 + 测距 + 去抖（31 gtest，含验收脚本）
│   └── fleet_agent/         # 执行代理（17 gtest，含验收脚本）
└── scripts/
    ├── install_deps.sh      # 依赖安装（幂等、逐项累计失败）
    ├── fault_injector.py    # 故障注入器（SIGSTOP 进程 / 挪动物块）
    ├── stop_sim.sh          # SIGTERM→SIGKILL 停仿真并清 Fast DDS 共享内存
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
- 两套 Nav2 的 10 个进程同时上线时，DDS 发现阶段会挤掉 lifecycle 的 `change_state` 响应（日志 `failed to send response ... change_state (timeout)`，bringup 永久卡在 `Configuring xxx`）；两套栈错开 10 s 启动后稳定

**定位与故障注入（M7）**
- gz 的 `DiffDrive` 里程计是轮速积分：`set_pose` 搬动机器人不会改 odom，搬完 TF 与真实位姿脱节；也不能用它模拟"原地打滑"，Nav2 会认为自己在正常前进
- 两台车的轮式里程计各自漂移（实测 1~3 m），跨车传递世界坐标必须统一到同一张地图 —— 改用每车一套 AMCL 后落点误差降到 0.1 m
- 冻结 `controller_server` 会连带断掉 lifecycle bond，整台车的 Nav2 被 deactivate；这是不可自愈故障，适合验收"改派"，不适合验收"故障车自愈"
- 收起来的物块要放场外（如 `(20,20)`）：放在场内角落时，8 m 外的小像素框会被单目测距当成 5 m 内的目标，清场与摆位期间不断冒出幽灵任务
- 改派后新车沿用旧车报的目标坐标，落点可能偏差米级；agent 到达对位点后若一直看不到目标，先倒车退出"贴脸"盲区、再原地转一圈搜索，仍找不到才报 `TARGET_LOST`
- 任务改派要校验归属：旧车恢复后补发的 `NAVIGATING/DONE` 和迟到的 action result 都可能落在已改派的任务上，必须比对 `assigned_robot` 与 `robot_task_`
- 机器人从离线恢复时要撤销它残留的导航目标，否则它会去抢已经改派给别的车的物块

**感知**
- tf2 禁止在回调里带超时查 TF（数据靠本节点执行器喂入，带超时等于自己等自己）；图像戳常比最新 TF 早/晚几毫秒，失败时退化为"最新位姿"
- 颜色阈值必须限制饱和度下限，否则灰白墙面与地面会被误判

**并发与进程管理**
- action 服务端回调组必须用 `Reentrant` + 总线互斥标志，否则新任务请求会排在长时间 `execute` 后面、等前一任务结束才被处理
- 终态只由 action result 承载：feedback 走话题、result 走服务响应，无顺序保证
- Nav2 节点在生命周期过渡卡住时会忽略 SIGTERM，残留进程与下次启动形成同名节点，`lifecycle_manager` 会一直卡住；用 `scripts/stop_sim.sh` 做 SIGTERM→SIGKILL 清理
- `bt_navigator` 默认只等 20 ms 连 `planner_server` 的 action server，节点刚激活时可能报 `compute_path_to_pose not available` 并拖垮整个 bringup，已把 `default_server_timeout` 提到 5000 ms

---

## 8. 已知限制

- 相机在软件渲染下实际约 4 Hz，视觉对位限速 0.15 m/s；激光约 2.3 Hz
- 感知每色只上报一个目标（当前场景三色各一块足够），偏轴测距会系统性偏小
- 两台车都接入 Nav2 + AMCL，`dispatch_robot_ids` 可控制派活范围
- AMCL 依赖手写地图与仿真激光，真实场地需要重新建图并标定；粒子滤波在长走廊场景仍需调参
- 调度中心的超时是"兜底"：正常失败由 agent 先报，看门狗只处理 agent 失联/卡死的情况
- 监控端尚未开发（M8），当前只能通过话题与 RViz 观察

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
| M6 | `fleet_manager` ROS 适配：队列/分配/超时重试接 action，服务与 RViz Marker | 完成 |
| M7 | 故障注入：目标丢失、导航失败、机器人离线、任务超时、重新分配 + AMCL 共享定位 | 完成 |
| M8 | Qt6 监控端：任务列表、机器人在线状态、告警、日志、相机流、rosbag 时间轴检索 | 计划 |
| M9 | 集成测试、rosbag 回放、Docker、CI、完整交付文档 | 计划 |

---

## 作者

赵威（enenr2） — 2027 届计算机科学与技术，方向：机器人上位机 / ROS2 系统集成。
