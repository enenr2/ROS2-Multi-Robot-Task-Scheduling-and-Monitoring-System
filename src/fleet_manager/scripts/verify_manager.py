#!/usr/bin/env python3
"""M6 验收：调度中心的自动发现、分配、执行与取消。

验收的是"没有人工介入也能跑完"这条链路：
  感知发现目标 -> 调度建任务 -> 选车分配 -> 直调 execute_task -> 收终态

判定项：
  1. 目标出现后调度自动建任务（无需 SubmitTask）
  2. 任务分配给最近的、允许派活的车（robot1；robot2 没有 Nav2，只监控）
  3. 任务走完 ASSIGNED -> NAVIGATING -> DOCKING -> DONE
  4. 执行结束后 robot1 停在物块前 0.6 m 左右
  5. 任务快照发布到 /fleet/tasks，RViz Marker 有输出
  6. 外部 CancelTask 能中止执行中的任务：任务转 CANCELED、车回到 IDLE 并停下

用法：
  source install/setup.bash
  python3 src/fleet_manager/scripts/verify_manager.py
"""
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

from fleet_msgs.msg import RobotStatus, Task
from fleet_msgs.srv import CancelTask, SubmitTask
from visualization_msgs.msg import Marker, MarkerArray

WORLD = 'fleet'
ROBOT = 'robot1'
BASE_FRAME = ROBOT + '_base_footprint'
BLUE = 'block_blue'
GREEN = 'block_green'
RED = 'block_red'
BLOCK_Z = 0.1
STANDOFF = 0.6
AHEAD = 3.0
OFFSET = math.radians(20.0)
# 三段内隔断的包围盒（与 worlds/fleet_tasks.sdf 一致）：用来判断视线是否被挡
WALLS = [(-3.0, 4.9, 3.0, 5.1), (-4.1, -3.0, -3.9, 3.0), (4.9, -5.5, 5.1, -0.5)]

STATE_NAMES = {0: 'PENDING', 1: 'ASSIGNED', 2: 'NAVIGATING', 3: 'DOCKING',
               4: 'DONE', 5: 'FAILED', 6: 'CANCELED'}
COLOR_NAMES = {1: 'RED', 2: 'GREEN', 3: 'BLUE'}
HOMES = {RED: (2.5, 1.5), GREEN: (4.0, -1.2), BLUE: (1.5, 3.0)}
PARK = {RED: (8.5, 7.5), GREEN: (8.5, 9.0), BLUE: (8.5, 8.5)}

FAILURES = []


def check(label, ok, detail=''):
    print(f'  [{"通过" if ok else "不通过"}] {label}{(" — " + detail) if detail else ""}')
    if not ok:
        FAILURES.append(label)


def point_in_wall(x, y, margin=0.0):
    """世界里的三段内隔断（与 worlds/fleet_tasks.sdf 一致）。

    margin 用来把"贴着墙但没进墙"的位置也排除掉：物块放在隔断端头 0.1 m
    时，另一台车绕到对面常常看不到它（相机 FOV 被墙切掉一半），对位阶段
    会误报 TARGET_LOST。
    """
    for x0, y0, x1, y1 in WALLS:
        if x0 - margin <= x <= x1 + margin and y0 - margin <= y <= y1 + margin:
            return True
    return False


def path_blocked(x0, y0, x1, y1):
    steps = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.05))
    for i in range(steps + 1):
        t = i / steps
        if point_in_wall(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t):
            return True
    return False


def choose_spot(rx, ry, yaw):
    """在车头方向附近挑一个"看得见且不被墙挡"的放置点。

    车停在哪儿由上一轮任务决定，不能假设它总在开阔地。内隔断挡住的
    目标既不会被相机发现，也会让后续导航绕远，所以先做几何检查。
    """
    for offset_deg in (0.0, 15.0, -15.0, 25.0, -25.0):
        for distance in (AHEAD, 2.5, 2.0):
            th = yaw + math.radians(offset_deg)
            bx = rx + distance * math.cos(th)
            by = ry + distance * math.sin(th)
            if abs(bx) > 9.3 or abs(by) > 9.3:
                continue
            if point_in_wall(bx, by, 0.6) or path_blocked(rx, ry, bx, by):
                continue
            return bx, by
    return None


def clear_active_tasks(probe, timeout=30.0):
    """清场：取消所有未终态任务并等车回到 IDLE。

    刚启动时场内的绿块就在 robot1 视野里，调度会自己开一单 —— 这是系统
    在正常工作，但会让"等某个新任务出现"的验收脚本撞上"车正忙"。
    所以先显式清场，再开始测。
    """
    canceled = 0
    for tid, msg in list(probe.latest.items()):
        if msg.state in (4, 5, 6):
            continue
        req = CancelTask.Request()
        req.task_id = tid
        req.reason = '验收前清场'
        probe.cancel_client.call_async(req)
        canceled += 1
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)
        st = probe.robot_status.get(ROBOT)
        active = any(m.state in (0, 1, 2, 3) for m in probe.latest.values())
        if not active and st is not None and st.state == RobotStatus.STATE_IDLE:
            return True
    return False


def set_pose(name, x, y, z=BLOCK_Z, yaw=0.0):
    req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    # 仿真满载时 gz 服务的响应会迟到，2s 超时偶发失败；重试一次比让整个
    # 验收脚本崩在摆位上更划算
    for attempt in range(2):
        out = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
                              '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                              '--timeout', '5000', '--req', req],
                             capture_output=True, text=True, timeout=20)
        if 'data: true' in out.stdout:
            return
        if attempt == 0:
            time.sleep(0.5)
    raise RuntimeError(f'set_pose({name}) 失败: {out.stdout}{out.stderr}')


class ManagerProbe(Node):
    def __init__(self):
        super().__init__('manager_verify', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.history = {}      # task_id -> [state, ...]
        self.latest = {}       # task_id -> Task
        self.marker_msgs = 0
        self.robot_status = {}

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Task, '/fleet/tasks', self.on_task, qos)
        self.create_subscription(RobotStatus, f'/{ROBOT}/robot_status',
                                 lambda m: self.robot_status.__setitem__(ROBOT, m), qos)
        self.create_subscription(MarkerArray, '/fleet/task_markers',
                                 self.on_markers, qos)
        self.submit_client = self.create_client(SubmitTask, '/fleet/submit_task')
        self.cancel_client = self.create_client(CancelTask, '/fleet/cancel_task')

    def on_task(self, msg):
        self.latest[msg.task_id] = msg
        h = self.history.setdefault(msg.task_id, [])
        if not h or h[-1] != msg.state:
            h.append(msg.state)

    def on_markers(self, msg):
        if any(m.action == Marker.ADD for m in msg.markers):
            self.marker_msgs += 1

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

    def wait_task(self, color, min_state, timeout):
        """等出现某颜色的任务并达到至少 min_state，返回 task_id"""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            for tid, msg in self.latest.items():
                if msg.target_color == color and msg.state >= min_state:
                    return tid
        return None

    def wait_state(self, task_id, state, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            msg = self.latest.get(task_id)
            if msg is not None and msg.state == state:
                return True
        return False

    def wait_state_in(self, task_id, states, timeout):
        # 等任务进入给定状态集合之一。不能简单用 state >= n：
        # FAILED(5) 也大于 NAVIGATING(2)，会把"立刻失败"误判成"开始执行"
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            msg = self.latest.get(task_id)
            if msg is not None and msg.state in states:
                return True
        return False


def main():
    rclpy.init()
    probe = ManagerProbe()
    probe.spin_for(1.0)

    print('=' * 78)
    print('M6 调度中心验收')
    print('=' * 78)

    if not probe.submit_client.wait_for_service(timeout_sec=10.0):
        print('/fleet/submit_task 不可用，检查 fleet_manager 是否已启动')
        return 1

    # ---- 1. 自动发现与执行 ----
    print('\n[1] 自动发现 -> 分配 -> 执行 -> 完成')
    for block, (px, py) in PARK.items():
        set_pose(block, px, py)
    probe.spin_for(2.0)

    # 场内原有目标会先被自动接单，先清场再开始
    check('清场：取消残留任务并等车空闲', clear_active_tasks(probe))
    probe.spin_for(1.0)

    rx, ry, ryaw = probe.robot_pose()
    spot = choose_spot(rx, ry, ryaw)
    if spot is None:
        print('  找不到既在视野内又不被墙挡的放置点')
        return 1
    bx, by = spot
    print(f'  物块放置: ({bx:.2f}, {by:.2f})，车在 ({rx:.2f}, {ry:.2f}, '
          f'{math.degrees(ryaw):.1f}°)')

    # 物块正面朝向车：否则相机看到的是"斜着的立方体"，轮廓宽度会虚高、
    # 测距系统性偏小（M4 实测偏轴 24° 偏小约 0.2 m），停车距离就会偏大
    set_pose(BLUE, bx, by, yaw=math.atan2(by - ry, bx - rx))
    task_id = probe.wait_task(3, 1, timeout=25.0)   # 3=BLUE，1=ASSIGNED
    check('目标出现后自动建任务并分配', task_id is not None,
          task_id if task_id else '25s 内没有可用任务')
    if task_id is None:
        return 1

    snapshot = probe.latest[task_id]
    check('分配给允许派活的车', snapshot.assigned_robot_id == ROBOT,
          f'assigned_robot_id={snapshot.assigned_robot_id or "(空)"}')

    done = probe.wait_state(task_id, 4, timeout=150.0)
    seq = ' -> '.join(STATE_NAMES.get(s, str(s)) for s in probe.history.get(task_id, []))
    print(f'  状态序列: {seq}')
    check('任务完成（DONE）', done)
    check('状态链完整',
          probe.history.get(task_id, []) == [0, 1, 2, 3, 4],
          f'实际 {seq}')
    check('RViz Marker 有输出', probe.marker_msgs > 0,
          f'收到 {probe.marker_msgs} 帧含 ADD 的 MarkerArray')

    rx2, ry2, _ = probe.robot_pose()
    dist = math.hypot(rx2 - bx, ry2 - by)
    # 车到物块中心 ≈ standoff + 半个物块 + 相机前移量 ≈ 0.73 m
    check('停车位置在物块前约 standoff', 0.55 <= dist <= 0.95,
          f'车到物块中心 {dist:.2f} m（期望约 {STANDOFF + 0.13:.2f} m）')

    # ---- 2. 服务注入 + 取消 ----
    # 这里不走感知，直接注入一个"不存在的目标"，专门验证服务与取消链路
    print('\n[2] 外部注入与取消')
    for block, (px, py) in PARK.items():
        set_pose(block, px, py)
    probe.spin_for(2.0)

    submit = SubmitTask.Request()
    submit.target_color = 3            # BLUE
    submit.target_pose.position.x = 7.0
    submit.target_pose.position.y = 7.0
    submit.target_pose.orientation.w = 1.0

    future = probe.submit_client.call_async(submit)
    rclpy.spin_until_future_complete(probe, future, timeout_sec=10.0)
    res = future.result()
    check('SubmitTask 创建任务', res is not None and res.accepted,
          res.message if res else '服务无响应')
    injected_id = res.task_id if res else None

    # 同色同位置再提交一次：应被去重拒绝
    future = probe.submit_client.call_async(submit)
    rclpy.spin_until_future_complete(probe, future, timeout_sec=10.0)
    res_dup = future.result()
    check('重复提交被去重拒绝', res_dup is not None and not res_dup.accepted,
          res_dup.message if res_dup else '服务无响应')

    if injected_id is None:
        return 1

    dispatch = probe.wait_state_in(injected_id, (2, 3), timeout=30.0)
    check('注入的任务被分配并开始执行', dispatch,
          STATE_NAMES.get(probe.latest.get(injected_id).state, '?')
          if injected_id in probe.latest else '?')

    req = CancelTask.Request()
    req.task_id = injected_id
    req.reason = '验收取消'
    future = probe.cancel_client.call_async(req)
    rclpy.spin_until_future_complete(probe, future, timeout_sec=10.0)
    res = future.result()
    check('CancelTask 返回成功', res is not None and res.canceled,
          res.message if res else '服务无响应')

    canceled = probe.wait_state(injected_id, 6, timeout=15.0)
    check('任务转为 CANCELED', canceled,
          STATE_NAMES.get(probe.latest.get(injected_id).state, '?')
          if injected_id in probe.latest else '?')

    idle = False
    end = time.time() + 10.0
    while time.time() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)
        st = probe.robot_status.get(ROBOT)
        if st is not None and st.state == RobotStatus.STATE_IDLE:
            idle = True
            break
    check('机器人回到 IDLE', idle,
          f'state={probe.robot_status.get(ROBOT).state if ROBOT in probe.robot_status else "?"}')

    p1 = probe.robot_pose()
    probe.spin_for(2.0)
    p2 = probe.robot_pose()
    moved = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
    check('取消后车已停下', moved < 0.15, f'2s 内位移 {moved:.3f} m')

    for block, (hx, hy) in HOMES.items():
        set_pose(block, hx, hy)
    print('\n已恢复三个物块到初始位置')

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
