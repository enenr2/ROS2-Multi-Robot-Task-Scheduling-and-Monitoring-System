#!/usr/bin/env python3
"""端到端冒烟：headless 仿真是否还能走完"注入任务 → 导航 → 对位 → DONE"。

CI 的常规 job 不跑 Gazebo（慢、易受环境影响），这个脚本给 nightly / 手动触发用：
  1. 等 /map、两车心跳、两套 Nav2 的 navigate_to_pose 就绪
  2. 向红色物块 (2.5, 1.5) 注入一个任务
  3. 断言任务在超时内进入 DONE，车回到 IDLE
  4. 失败时打印当前任务表与机器人心跳，便于定位是感知、Nav2 还是调度的问题

用法：
    ros2 launch fleet_bringup fleet_sim.launch.py gui:=false rviz:=false &
    python3 scripts/smoke_sim.py --timeout 180
"""
import argparse
import math
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from fleet_msgs.msg import RobotStatus, Task, TaskStatus
from fleet_msgs.srv import SubmitTask
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

ROBOTS = ('robot1', 'robot2')
# 把蓝色物块挪到一个"没有历史任务"的空位再注入：否则保留期内的去重会直接
# 复用旧任务，冒烟就退化成"读一个 DONE 快照"
SMOKE_BLOCK = 'block_blue'
SMOKE_SPOT = (4.0, 4.0)
COLOR_BLUE = 3
STATE_NAVIGATING = 2
STATE_DOCKING = 3
STATE_DONE = 4


class Probe(Node):
    def __init__(self):
        super().__init__('fleet_smoke')
        self.tasks = {}
        self.robots = {}
        self.task_states = {}
        self.map_received = False
        self.lock = threading.RLock()

        self.create_subscription(Task, '/fleet/tasks', self._on_task, 20)
        self.create_subscription(TaskStatus, '/robot1/task_status', self._on_status, 50)
        self.create_subscription(TaskStatus, '/robot2/task_status', self._on_status, 50)
        self.create_subscription(RobotStatus, '/robot1/robot_status', self._on_robot, 20)
        self.create_subscription(RobotStatus, '/robot2/robot_status', self._on_robot, 20)
        self.create_subscription(OccupancyGrid, '/map', self._on_map,
                                 QoSProfile(depth=1,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.submit = self.create_client(SubmitTask, '/fleet/submit_task')
        self.nav = {name: ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose')
                    for name in ROBOTS}

    def _on_task(self, msg):
        with self.lock:
            self.tasks[msg.task_id] = msg

    def _on_robot(self, msg):
        with self.lock:
            self.robots[msg.robot_id] = msg

    def _on_status(self, msg):
        with self.lock:
            self.task_states.setdefault(msg.task_id, set()).add(msg.state)

    def states(self, task_id):
        with self.lock:
            return set(self.task_states.get(task_id, set()))

    def _on_map(self, _msg):
        with self.lock:
            self.map_received = True

    def snapshot(self, task_id):
        with self.lock:
            return self.tasks.get(task_id)

    def dump(self):
        with self.lock:
            print('--- 任务表 ---')
            for task in self.tasks.values():
                print(f'  {task.task_id} color={task.target_color} state={task.state} '
                      f'robot={task.assigned_robot_id or "-"} '
                      f'target=({task.target_pose.position.x:.2f}, '
                      f'{task.target_pose.position.y:.2f}) reason={task.failure_reason}')
            print('--- 机器人 ---')
            for name, status in self.robots.items():
                print(f'  {name} state={status.state} seq={status.heartbeat_seq} '
                      f'pose=({status.pose.position.x:.2f}, '
                      f'{status.pose.position.y:.2f}) task="{status.current_task}" '
                      f'done={status.tasks_done} failed={status.tasks_failed}')


def move_block(world, name, x, y):
    """用 gz set_pose 把物块挪到空位（world 默认 fleet）。"""
    request = (f'name: "{name}", position: {{x: {x}, y: {y}, z: 0.1}}, '
               f'orientation: {{w: 1.0}}')
    try:
        result = subprocess.run(
            ['gz', 'service', '-s', f'/world/{world}/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', request],
            capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.TimeoutExpired) as ex:
        print(f'[EE] 调用 gz set_pose 失败：{ex}', file=sys.stderr)
        return False
    if 'data: true' not in result.stdout:
        print(f'[EE] set_pose 未成功：{result.stdout}{result.stderr}', file=sys.stderr)
        return False
    return True


def wait_until(probe, predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        with probe.lock:
            if predicate():
                return True
        time.sleep(0.1)
    print(f'[EE] 等待超时：{description}', file=sys.stderr)
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=180.0, help='任务完成超时（秒）')
    parser.add_argument('--boot-timeout', type=float, default=120.0, help='仿真就绪超时（秒）')
    parser.add_argument('--world', default='fleet', help='gz 世界名')
    args = parser.parse_args()

    rclpy.init()
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    threading.Thread(target=executor.spin, daemon=True).start()

    ok = True
    task_id = ''

    try:
        if not wait_until(probe, lambda: probe.map_received, args.boot_timeout, '/map'):
            ok = False
        if ok and not wait_until(
                probe, lambda: all(name in probe.robots for name in ROBOTS),
                args.boot_timeout, '两车心跳'):
            ok = False
        if ok:
            for name in ROBOTS:
                if not probe.nav[name].wait_for_server(timeout_sec=args.boot_timeout):
                    print(f'[EE] {name} 的 navigate_to_pose 不可用', file=sys.stderr)
                    ok = False
        if ok and not probe.submit.wait_for_service(timeout_sec=args.boot_timeout):
            print('[EE] /fleet/submit_task 不可用', file=sys.stderr)
            ok = False

        if ok:
            print(f'[..] 把 {SMOKE_BLOCK} 挪到 {SMOKE_SPOT} 并注入任务')
            if not move_block(args.world, SMOKE_BLOCK, *SMOKE_SPOT):
                ok = False
            else:
                time.sleep(2.0)   # 等物理稳定与感知重新发现

        if ok:
            request = SubmitTask.Request()
            request.target_color = COLOR_BLUE
            request.target_pose.position.x = SMOKE_SPOT[0]
            request.target_pose.position.y = SMOKE_SPOT[1]
            request.target_pose.orientation.w = 1.0
            future = probe.submit.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _f: done.set())
            if not done.wait(20.0):
                print('[EE] SubmitTask 调用超时', file=sys.stderr)
                ok = False
            else:
                response = future.result()
                print(f'[..] SubmitTask accepted={response.accepted} msg={response.message}')
                if response.accepted:
                    task_id = response.task_id
                else:
                    # 感知可能已经先建了同一个目标的单，直接跟踪已有任务
                    with probe.lock:
                        for task in probe.tasks.values():
                            dist = math.hypot(
                                task.target_pose.position.x - SMOKE_SPOT[0],
                                task.target_pose.position.y - SMOKE_SPOT[1])
                            if task.target_color == COLOR_BLUE and dist < 0.8:
                                task_id = task.task_id
                                break
                    if not task_id:
                        ok = False
                    else:
                        print(f'[..] 复用已有任务 {task_id}')

        if ok:
            print(f'[..] 等待 {task_id} 进入 DONE（上限 {args.timeout:.0f}s）')
            ok = wait_until(probe, lambda: probe.snapshot(task_id) is not None
                            and probe.snapshot(task_id).state == STATE_DONE,
                            args.timeout, f'{task_id} DONE')

        if ok:
            # 只认"真的走过导航与对位"：复用一条历史 DONE 快照不算通过
            seen = probe.states(task_id)
            if STATE_DOCKING not in seen:
                print(f'[EE] {task_id} 没有进入对位（观察到状态 {sorted(seen)}）',
                      file=sys.stderr)
                ok = False
            else:
                print('[..] 已观察到 NAVIGATING → DOCKING → DONE')
    finally:
        if not ok:
            probe.dump()
        executor.shutdown()
        probe.destroy_node()
        rclpy.try_shutdown()

    if ok:
        print('[OK] 冒烟通过：注入任务已走完导航与对位')
        return 0
    print('[EE] 冒烟失败', file=sys.stderr)
    return 1


if __name__ == '__main__':
    sys.exit(main())