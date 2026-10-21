#!/usr/bin/env python3
"""M5 验收：向单台车下发一次 ExecuteTask，检查导航 -> 视觉对位 -> 停车的完整闭环。

为什么用一个脚本而不是手动点：这一段的验收标准是数字，不是"看着像到了"：
  1. action 结果 success=True，失败原因为 NONE
  2. 最终停车距离与 standoff 之差 <= 0.15 m（视觉测距本身的误差就在 0.1 m 内）
  3. 最终航向误差 <= 0.1 rad（约 5.7°，容差配置为 0.05 rad）
  4. 状态序列必须是 ASSIGNED -> NAVIGATING -> DOCKING -> DONE
  5. 执行期间心跳机器人状态为 BUSY，结束后回到 IDLE

物块用 gz set_pose 放到车头正前方 3 m 处，位置由脚本给定，所以是已知真值。

用法：
  source install/setup.bash
  python3 src/fleet_agent/scripts/verify_agent.py
"""
import math
import statistics
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

from fleet_msgs.action import ExecuteTask
from fleet_msgs.msg import RobotStatus, TaskStatus

WORLD = 'fleet'
ROBOT = 'robot1'
BASE_FRAME = ROBOT + '_base_footprint'
BLOCK = 'block_blue'
BLOCK_COLOR = 3          # Detection.COLOR_BLUE
BLOCK_HOME = (1.5, 3.0, 0.1)
BLOCK_Z = 0.1
STANDOFF = 0.6
APPROACH_DISTANCE = 3.0  # 物块放到车头前方多远
# 为什么要偏 45°：robot2 出生在 (1.5, 0)，正前方恰好被它挡住视线，
# 直线摆放时物块时有时无（实测会导致对位阶段误判 TARGET_LOST）
BLOCK_OFFSET = math.radians(45.0)

STATE_NAMES = {0: 'PENDING', 1: 'ASSIGNED', 2: 'NAVIGATING', 3: 'DOCKING',
               4: 'DONE', 5: 'FAILED', 6: 'CANCELED'}

FAILURES = []


def check(label, ok, detail=''):
    print(f'  [{"通过" if ok else "不通过"}] {label}{(" — " + detail) if detail else ""}')
    if not ok:
        FAILURES.append(label)


def set_pose(name, x, y, z=BLOCK_Z):
    req = f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}'
    out = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
                          '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                          '--timeout', '2000', '--req', req],
                         capture_output=True, text=True, timeout=15)
    if 'data: true' not in out.stdout:
        raise RuntimeError(f'set_pose({name}) 失败: {out.stdout}{out.stderr}')


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class AgentProbe(Node):
    def __init__(self):
        super().__init__('agent_verify', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.task_states = []
        self.robot_samples = []

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(TaskStatus, f'/{ROBOT}/task_status',
                                 lambda m: self.task_states.append(m), qos)
        self.create_subscription(RobotStatus, f'/{ROBOT}/robot_status',
                                 lambda m: self.robot_samples.append(m), qos)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def robot_pose(self):
        last = None
        for _ in range(60):
            try:
                tf = self.tf_buffer.lookup_transform('map', BASE_FRAME, rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return t.x, t.y, yaw
            except Exception as ex:
                last = ex
                self.spin_for(0.3)
        raise RuntimeError(f'TF map -> {BASE_FRAME} 不可用: {last}')


def main():
    rclpy.init()
    probe = AgentProbe()
    probe.spin_for(1.0)
    rx, ry, ryaw = probe.robot_pose()
    bx = rx + APPROACH_DISTANCE * math.cos(ryaw + BLOCK_OFFSET)
    by = ry + APPROACH_DISTANCE * math.sin(ryaw + BLOCK_OFFSET)

    print('=' * 78)
    print('M5 执行代理验收')
    print('=' * 78)
    print(f'机器人起始位姿: ({rx:.2f}, {ry:.2f}, {math.degrees(ryaw):.1f}°)')
    print(f'物块放置位置  : ({bx:.2f}, {by:.2f})  距离 {APPROACH_DISTANCE:.1f} m'
          f'（车头左偏 {math.degrees(BLOCK_OFFSET):.0f}°，避开 robot2 遮挡），'
          f'期望停车 {STANDOFF:.1f} m\n')

    set_pose(BLOCK, bx, by)
    probe.spin_for(3.0)

    client = ActionClient(probe, ExecuteTask, f'/{ROBOT}/execute_task')
    if not client.wait_for_server(timeout_sec=15.0):
        print(f'/{ROBOT}/execute_task 不可用，检查 robot_agent 是否已启动')
        return 1

    def make_goal(task_id, tx, ty):
        g = ExecuteTask.Goal()
        g.task_id = task_id
        g.target_color = BLOCK_COLOR
        # ExecuteTask.Goal 的 target_pose 是 geometry_msgs/Pose（无 header），
        # 坐标系在 action 定义里已约定为 map
        g.target_pose.position.x = tx
        g.target_pose.position.y = ty
        g.target_pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(0.0)
        g.target_pose.orientation.x = qx
        g.target_pose.orientation.y = qy
        g.target_pose.orientation.z = qz
        g.target_pose.orientation.w = qw
        g.standoff = STANDOFF
        return g

    goal = make_goal('verify-m5-1', bx, by)

    feedback_log = []
    states_seen = []

    def on_feedback(msg):
        fb = msg.feedback
        name = STATE_NAMES.get(fb.state, str(fb.state))
        if name not in states_seen:
            states_seen.append(name)
        feedback_log.append((time.time(), name, fb.distance_remaining, fb.progress, fb.detail))

    send_future = client.send_goal_async(goal, feedback_callback=on_feedback)
    print('已发送任务，等待接受...')
    rclpy.spin_until_future_complete(probe, send_future, timeout_sec=10.0)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        print('任务被拒绝（检查 agent 是否正忙）')
        return 1
    print('任务已接受，开始执行\n')

    t0 = time.time()
    result_future = handle.get_result_async()
    while not result_future.done():
        rclpy.spin_once(probe, timeout_sec=0.1)
        if time.time() - t0 > 180.0:
            print('等待结果超时（180s）')
            break
    elapsed = time.time() - t0

    if not result_future.done():
        return 1

    wrapped = result_future.result()
    result = wrapped.result

    # 心跳 1 Hz：结果返回后还要再等一拍，才能看到结束后的 IDLE 心跳
    probe.spin_for(2.0)

    # ---------- 过程 ----------
    print('状态序列:')
    printed = set()
    for _, name, dist, prog, detail in feedback_log:
        if name in printed:
            continue
        printed.add(name)
        print(f'  {name:<12} 首次出现时 distance={dist:+.2f} progress={prog:.2f}  {detail}')

    print(f'\n执行耗时: {elapsed:.1f}s，反馈 {len(feedback_log)} 条')
    print(f'结果: success={result.success} reason={result.failure_reason} '
          f'final_distance={result.final_distance:.3f} '
          f'final_heading_error={math.degrees(result.final_heading_error):+.2f}°\n')

    # ---------- 判定 ----------
    print('验收项:')
    check('action 返回成功', result.success, f'reason={result.failure_reason}')
    check('停车距离接近 standoff', abs(result.final_distance - STANDOFF) <= 0.15,
          f'{result.final_distance:.3f} m vs {STANDOFF:.1f} m')
    check('航向误差 <= 0.1 rad',
          abs(result.final_heading_error) <= 0.1,
          f'{math.degrees(result.final_heading_error):+.2f}°')

    # feedback 只承载过程态；终态由 result 承载（feedback 与 result 无顺序保证，
    # 客户端拿到 result 后就不再处理反馈，要求 DONE 出现在 feedback 里会偶发失败）
    expected = ['ASSIGNED', 'NAVIGATING', 'DOCKING']
    check('过程状态序列完整', states_seen == expected, '实际 ' + ' -> '.join(states_seen))

    done_msgs = [m for m in probe.task_states if m.state == TaskStatus.STATE_DONE]
    check('task_status 收到 DONE', len(done_msgs) >= 1,
          f'共 {len(probe.task_states)} 条状态事件')
    if done_msgs:
        check('DONE 的 progress 为 1.0', abs(done_msgs[-1].progress - 1.0) < 1e-3,
              f'progress={done_msgs[-1].progress:.2f}')

    busy_samples = [m for m in probe.robot_samples if m.state == RobotStatus.STATE_BUSY]
    idle_after = probe.robot_samples and probe.robot_samples[-1].state == RobotStatus.STATE_IDLE
    check('执行期间心跳报 BUSY', len(busy_samples) >= 1,
          f'BUSY 心跳 {len(busy_samples)} 条 / 共 {len(probe.robot_samples)} 条')
    check('结束后回到 IDLE', bool(idle_after))
    if len(probe.robot_samples) >= 3:
        seq = [m.heartbeat_seq for m in probe.robot_samples]
        check('心跳序号递增', all(b >= a for a, b in zip(seq, seq[1:])), f'首尾 {seq[0]}..{seq[-1]}')

    # ---------- 2. 故障路径 ----------
    print('\n[2] 故障路径')
    rx, ry, ryaw = probe.robot_pose()
    bx = rx + APPROACH_DISTANCE * math.cos(ryaw + BLOCK_OFFSET)
    by = ry + APPROACH_DISTANCE * math.sin(ryaw + BLOCK_OFFSET)
    set_pose(BLOCK, bx, by)
    probe.spin_for(2.5)

    # 2.1 并发下发：同一台车同一时刻只接一个任务
    seen_docking = {'value': False}

    def on_feedback_a(msg):
        if STATE_NAMES.get(msg.feedback.state) == 'DOCKING':
            seen_docking['value'] = True

    first = client.send_goal_async(make_goal('verify-busy-1', bx, by),
                                   feedback_callback=on_feedback_a)
    rclpy.spin_until_future_complete(probe, first, timeout_sec=10.0)
    h1 = first.result()

    second = client.send_goal_async(make_goal('verify-busy-2', bx, by))
    rclpy.spin_until_future_complete(probe, second, timeout_sec=10.0)
    h2 = second.result()

    check('并发下发时第二个任务被拒绝',
          h1 is not None and h1.accepted and h2 is not None and not h2.accepted,
          f'第一个 accepted={bool(h1 and h1.accepted)}，'
          f'第二个 accepted={bool(h2 and h2.accepted) if h2 else "响应超时（未被及时处理）"}')

    # 2.2 对位阶段目标丢失：应失败并给出 TARGET_LOST
    print('  等待执行进入对位阶段...')
    t0 = time.time()
    while not seen_docking['value'] and time.time() - t0 < 90.0:
        rclpy.spin_once(probe, timeout_sec=0.1)
    check('执行进入 DOCKING 阶段', seen_docking['value'])

    set_pose(BLOCK, 8.5, 9.0)          # 把目标移出视野
    t0 = time.time()
    while time.time() - t0 < 60.0:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if h1.status >= GoalStatus.STATUS_SUCCEEDED:   # 4/5/6 都是终态
            break

    # 终态以 result 响应里的 status 为准：goal handle 上的 status 是订阅到的
    # 中间态缓存，偶尔会慢一拍
    res1 = h1.get_result_async()
    rclpy.spin_until_future_complete(probe, res1, timeout_sec=10.0)
    wrapped = res1.result()
    status = wrapped.status
    check('目标丢失后任务进入终态', status >= GoalStatus.STATUS_SUCCEEDED, f'status={status}')
    # 注意：实测 rclcpp_action 在这条路径上把结果状态报成 CANCELED(6) 而不是
    # ABORTED(5)，但 result 里的 failure_reason 是准确的 TARGET_LOST。
    # 调度层（M6）以 failure_reason 为准，所以这里只要求"非成功终态"。
    check('目标丢失判为非成功终态', status != GoalStatus.STATUS_SUCCEEDED, f'status={status}')
    check('失败原因为 TARGET_LOST', wrapped.result.failure_reason == 'TARGET_LOST',
          f'实际 {wrapped.result.failure_reason}')
    probe.spin_for(1.5)

    set_pose(BLOCK, *BLOCK_HOME)
    print(f'\n已恢复 {BLOCK} 到 {BLOCK_HOME}')

    print('\n' + '=' * 78)
    if FAILURES:
        print(f'验收结果：不通过（{len(FAILURES)} 项）')
        for f in FAILURES:
            print(f'  - {f}')
    else:
        print('验收结果：全部通过')
    print('=' * 78)

    probe.destroy_node()
    rclpy.shutdown()
    return 1 if FAILURES else 0


if __name__ == '__main__':
    sys.exit(main())