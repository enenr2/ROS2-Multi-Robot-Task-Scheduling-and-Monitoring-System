#!/usr/bin/env python3
"""M7 验收：四类故障注入与调度层的恢复行为。

每个场景都是"真实跑一遍"，不是看日志猜：
  A 机器人离线   -> 心跳超时 ROBOT_UNREACHABLE -> 改派 -> 完成
  B 任务超时     -> 看门狗 NAV_TIMEOUT -> 取消 action -> 改派 -> 完成
  D 目标丢失     -> agent 报 TARGET_LOST -> 重试一次仍失败 -> FAILED
  C 导航栈卡死   -> agent 报 NAV_ABORTED -> 改派 -> 完成

B 是"任务超出预算"：在线把 nav_timeout 缩到 2 s，正常导航跑不完就会被
看门狗判超时；检测到重排后立刻恢复 100 s，避免改派的那台车也被同一预算卡死。

C 放在最后：冻结 controller_server 会让该车 Nav2 整栈被 lifecycle 下线，
是四个场景里唯一"故障车不会自愈"的，跑完这台车就退出服务了。

故障用 scripts/fault_injector.py 注入（SIGSTOP / 挪物块），不改被测代码。

用法：
  source install/setup.bash
  python3 src/fleet_manager/scripts/verify_faults.py [--scenario A|B|C|D]
"""
import argparse
import math
import os
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_manager import (  # noqa: E402  复用几何与放置逻辑
    ROBOT, STATE_NAMES, choose_spot, set_pose)

# 收起来的物块放场外：放在场内角落（8.5, 7.5）时，8 m 外的小像素框会被
# 测距当成 5 m 内的目标，清场和摆位期间不断冒出幽灵任务，把车拖走
PARK_FAR = {'block_red': (20.0, 20.0), 'block_green': (22.0, 20.0),
            'block_blue': (20.0, 22.0)}

from fleet_msgs.msg import RobotStatus, Task  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
INJECTOR = os.path.join(REPO_ROOT, 'scripts', 'fault_injector.py')

FAILURES = []


def check(label, ok, detail=''):
    print(f'  [{"通过" if ok else "不通过"}] {label}{(" — " + detail) if detail else ""}')
    if not ok:
        FAILURES.append(label)


class Probe(Node):
    def __init__(self):
        super().__init__('fault_verify', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest = {}
        self.history = {}
        self.robot_status = {}
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Task, '/fleet/tasks', self.on_task, qos)
        for robot in ('robot1', 'robot2'):
            self.create_subscription(
                RobotStatus, f'/{robot}/robot_status',
                lambda m, r=robot: self.robot_status.__setitem__(r, m), qos)
        self.param_client = self.create_client(
            SetParametersAtomically, '/fleet_manager/set_parameters_atomically')
        self.get_param_client = self.create_client(
            GetParameters, '/fleet_manager/get_parameters')

    def on_task(self, msg):
        self.latest[msg.task_id] = msg
        h = self.history.setdefault(msg.task_id, [])
        if not h or h[-1] != msg.state:
            h.append(msg.state)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pose(self, robot):
        for _ in range(60):
            try:
                tf = self.tf_buffer.lookup_transform('map', robot + '_base_footprint',
                                                     rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return t.x, t.y, yaw
            except Exception:
                self.spin_for(0.3)
        raise RuntimeError(f'TF 拿不到 {robot} 的位姿')


def set_param(probe, name, value):
    # 刚启动时参数服务可能还没被发现，直接 call_async 会拿不到结果
    probe.param_client.wait_for_service(timeout_sec=10.0)
    req = SetParametersAtomically.Request()
    p = Parameter()
    p.name = name
    p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
    req.parameters = [p]
    future = probe.param_client.call_async(req)
    rclpy.spin_until_future_complete(probe, future, timeout_sec=10.0)
    res = future.result()
    return res is not None and res.result.successful


def get_param(probe, name, default=None):
    probe.get_param_client.wait_for_service(timeout_sec=10.0)
    req = GetParameters.Request()
    req.names = [name]
    future = probe.get_param_client.call_async(req)
    rclpy.spin_until_future_complete(probe, future, timeout_sec=10.0)
    res = future.result()
    if res is None or not res.values:
        return default
    v = res.values[0]
    if v.type == ParameterType.PARAMETER_DOUBLE:
        return v.double_value
    if v.type == ParameterType.PARAMETER_INTEGER:
        return float(v.integer_value)
    return default


def clear_tasks(probe, timeout=30.0):
    from fleet_msgs.srv import CancelTask
    client = probe.create_client(CancelTask, '/fleet/cancel_task')
    client.wait_for_service(timeout_sec=5.0)
    for tid, msg in list(probe.latest.items()):
        if msg.state in (4, 5, 6):
            continue
        req = CancelTask.Request()
        req.task_id = tid
        req.reason = '验收清场'
        client.call_async(req)
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if not any(m.state in (0, 1, 2, 3) for m in probe.latest.values()):
            return True
    return False


def start_injector(args):
    # 不接管 stdout：注入器失败时会打印堆栈，吞进 PIPE 就再也看不到了
    return subprocess.Popen([sys.executable, INJECTOR] + args)


def wait_for(probe, predicate, timeout, label):
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if predicate():
            return True
    print(f'  （等待超时：{label}）')
    return False


# 注意：不要用 gz set_pose 搬动机器人。实测 tb3 的 /robotN/odom 由轮速积分
# 得到，set_pose 只改物理位姿、不改里程计，搬完之后 TF 与真实位置脱节，
# 感知算出的 world_pose、Nav2 的定位全部错位（表现为"目标在 6,-5 却看不见"）。
# 机器人的位置只能由它自己导航决定。


def idle_robots(probe, timeout=25.0):
    """等两台车都不在执行任务。

    必须先确认这一点再摆位：只要还有车在跑导航，set_pose 把车挪回去，
    Nav2 下一秒就把它开走 —— 目标点和车头朝向都会漂，验收结果就不可复现。
    """
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)
        if all(m.state == 0 and not m.current_task for m in probe.robot_status.values()) \
                and len(probe.robot_status) == 2:
            return True
    return False


def stage(probe, robot, x, y, yaw):
    """用 Nav2 把车开到固定机位。

    不能用 set_pose 瞬移：odom 是轮速积分出来的，不会跟着跳，搬完 TF 就废了。
    走 Nav2 目标点虽然慢十几秒，但里程计、代价地图、定位全程自洽。
    """
    cx, cy, cyaw = probe.pose(robot)
    if math.hypot(x - cx, y - cy) < 0.35 and abs(math.atan2(math.sin(cyaw - yaw),
                                                           math.cos(cyaw - yaw))) < 0.25:
        return True

    client = ActionClient(probe, NavigateToPose, f'/{robot}/navigate_to_pose')
    if not client.wait_for_server(timeout_sec=10.0):
        print(f'  （{robot} 的 Nav2 不可用，放弃摆位）')
        return False

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.header.stamp = probe.get_clock().now().to_msg()
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(probe, send, timeout_sec=15.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        print(f'  （{robot} 拒绝摆位目标）')
        return False

    result = handle.get_result_async()
    rclpy.spin_until_future_complete(probe, result, timeout_sec=90.0)
    if not result.done():
        print(f'  （{robot} 摆位超时）')
        return False
    # 注意：目标被任务抢占时 future 也会完成，必须看状态码，否则会把
    # "被抢占"误报成"已就位"（实测过，后面所有摆位都建立在错误前提上）
    status = result.result().status
    if status != GoalStatus.STATUS_SUCCEEDED:
        print(f'  （{robot} 摆位被中断，status={status}）')
        return False

    cx, cy, _ = probe.pose(robot)
    if math.hypot(x - cx, y - cy) > 0.5:
        print(f'  （{robot} 报就位但实际在 ({cx:.2f}, {cy:.2f})）')
        return False
    print(f'  （{robot} 已就位 ({cx:.2f}, {cy:.2f})）')
    return True


def prepare(probe):
    """收走物块 -> 清场 -> 等车停下 -> 开车到固定机位"""
    for block, (px, py) in PARK_FAR.items():
        set_pose(block, px, py)      # 先把诱饵收走，免得清场期间又建新任务

    # 清场必须真的清干净：只要还有车在跑，后面 Nav2 摆位会被 leftover 任务
    # 抢走，车停在墙边，目标点就再也挑不出来（实测 D 场景整段作废）
    ok = False
    for _ in range(3):
        clear_tasks(probe, timeout=20.0)
        if idle_robots(probe, timeout=10.0):
            ok = True
            break
    print(f'  （清场：{"两台车已空闲" if ok else "仍有车在执行任务"}）')
    if not ok:
        raise RuntimeError('清场失败，仍有车在执行任务')
    for block, (px, py) in PARK_FAR.items():
        set_pose(block, px, py)

    # robot1 朝 +x，目标固定摆在正前方 3 m；robot2 在侧后方待命，
    # 不挡视线、也不占目标点
    for robot, x, y in (('robot1', 0.5, 0.0), ('robot2', 1.5, 3.0)):
        for _ in range(2):
            if stage(probe, robot, x, y, 0.0):
                break
            clear_tasks(probe, timeout=10.0)   # 被抢占过，清一次再试
    probe.spin_for(1.0)


def place_in_view(probe, block, robot):
    x, y, yaw = probe.pose(robot)
    spot = choose_spot(x, y, yaw)
    if spot is None:
        raise RuntimeError('找不到可放置点')
    print(f'  （{robot} 在 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)，'
          f'{block} 放到 ({spot[0]:.2f}, {spot[1]:.2f})）')
    set_pose(block, spot[0], spot[1], yaw=math.atan2(spot[1] - y, spot[0] - x))
    return spot


def scenario_a(probe):
    print('\n[A] 机器人离线 -> 心跳超时 -> 任务改派')
    prepare(probe)
    known = set(probe.latest)
    place_in_view(probe, 'block_blue', ROBOT)
    task_id = wait_task(probe, 3, (1, 2), 30.0, '蓝色目标派单', exclude=known)
    if task_id is None:
        check('目标被发现并派单', False, '未出现')
        return
    check('目标被发现并派单', True, task_id)
    first_robot = probe.latest[task_id].assigned_robot_id
    print(f'  首次派给 {first_robot}')

    proc = start_injector(['robot-offline', '--robot', first_robot, '--hold', '25'])
    reassigned = wait_for(probe, lambda: probe.latest[task_id].retry_count > 0,
                          20.0, '任务被重排')
    check('离线后任务被重排', reassigned,
          f"retry={probe.latest[task_id].retry_count} "
          f"reason={probe.latest[task_id].failure_reason}")
    new_robot = wait_reassigned(probe, task_id, first_robot)
    check('改派到另一台车', new_robot not in ('', first_robot), f'{first_robot} -> {new_robot}')

    done = wait_for(probe, lambda: probe.latest[task_id].state == 4, 120.0, '改派后完成')
    check('改派后任务完成', done,
          f'state={STATE_NAMES.get(probe.latest[task_id].state)}')
    check('失败原因记录为心跳超时', probe.latest[task_id].failure_reason == 'ROBOT_UNREACHABLE',
          probe.latest[task_id].failure_reason)
    proc.wait(timeout=40)


def wait_task(probe, color, states, timeout, label, exclude=None):
    """等指定颜色的任务进入给定状态，返回 task_id（无则 None）。

    exclude 用来排除场景开始前就存在的任务：清场不干净时，上一轮的旧任务
    恰好也在 DOCKING，脚本会把它当成这一轮的目标，整段判定就错了。
    """
    exclude = exclude or set()
    task_id = None
    end = time.time() + timeout
    while time.time() < end and task_id is None:
        rclpy.spin_once(probe, timeout_sec=0.1)
        for tid, m in probe.latest.items():
            if tid in exclude:
                continue
            if m.target_color == color and m.state in states:
                task_id = tid
    print(f'  （{label}：{task_id or "未出现"}）')
    return task_id


def wait_reassigned(probe, task_id, first, timeout=15.0):
    """等任务真的落到另一台车上。

    重排是"retry_count+1"和"改派"两步，快照先发出 retry 再发 assignment，
    立刻读 assigned_robot_id 往往还是空串；多等几拍才拿得到真实归属。
    """
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(probe, timeout_sec=0.1)
        assigned = probe.latest[task_id].assigned_robot_id
        if assigned not in ('', first):
            return assigned
    return probe.latest[task_id].assigned_robot_id


def scenario_b(probe):
    print('\n[B] 任务超出时间预算 -> 看门狗 NAV_TIMEOUT -> 取消并改派')
    # 顺序很重要：先清场再改参数。反过来的话，刚启动时场内那块红目标
    # 会被 2s 的预算判超时，白白给 robot2 记一次失败冷却，改派就落空了
    prepare(probe)
    check('在线把 nav_timeout 缩到 2s', set_param(probe, 'nav_timeout', 2.0))
    check('参数确认生效', get_param(probe, 'nav_timeout') == 2.0,
          f'{get_param(probe, "nav_timeout")}')
    known = set(probe.latest)
    place_in_view(probe, 'block_red', ROBOT)

    task_id = wait_task(probe, 1, (1, 2), 30.0, '红色目标派单', exclude=known)
    if task_id is None:
        check('目标被发现并派单', False, '未出现')
        set_param(probe, 'nav_timeout', 100.0)
        return
    check('目标被发现并派单', True, task_id)
    first_robot = probe.latest[task_id].assigned_robot_id
    print(f'  首次派给 {first_robot}')

    timed_out = wait_for(probe, lambda: probe.latest[task_id].retry_count > 0, 20.0,
                         '看门狗判定超时')
    # 立刻恢复预算，否则改派的那台车也会被 2s 卡死
    check('恢复 nav_timeout 默认值', set_param(probe, 'nav_timeout', 100.0))
    check('看门狗判定任务超时并重排', timed_out,
          f"retry={probe.latest[task_id].retry_count} "
          f"reason={probe.latest[task_id].failure_reason}")
    check('失败原因为 NAV_TIMEOUT', probe.latest[task_id].failure_reason == 'NAV_TIMEOUT',
          probe.latest[task_id].failure_reason)
    new_robot = wait_reassigned(probe, task_id, first_robot)
    check('改派到另一台车', new_robot not in ('', first_robot), f'{first_robot} -> {new_robot}')

    done = wait_for(probe, lambda: probe.latest[task_id].state == 4, 150.0, '改派后完成')
    check('改派后任务完成', done, STATE_NAMES.get(probe.latest[task_id].state, '?'))


def scenario_c(probe):
    print('\n[C] 导航栈卡死 -> NAV_ABORTED -> 取消并改派（最后跑）')
    prepare(probe)
    known = set(probe.latest)
    place_in_view(probe, 'block_blue', ROBOT)

    task_id = wait_task(probe, 3, (1, 2), 30.0, '蓝色目标派单', exclude=known)
    if task_id is None:
        check('目标被发现并派单', False, '未出现')
        return
    check('目标被发现并派单', True, task_id)
    first_robot = probe.latest[task_id].assigned_robot_id
    print(f'  首次派给 {first_robot}')

    proc = start_injector(['nav-failure', '--robot', first_robot, '--hold', '35'])
    aborted = wait_for(probe, lambda: probe.latest[task_id].retry_count > 0, 60.0,
                       '导航栈报失败')
    check('导航失败触发重排', aborted,
          f"retry={probe.latest[task_id].retry_count} "
          f"reason={probe.latest[task_id].failure_reason}")
    check('失败原因为 NAV_ABORTED',
          probe.latest[task_id].failure_reason == 'NAV_ABORTED',
          probe.latest[task_id].failure_reason)
    new_robot = wait_reassigned(probe, task_id, first_robot)
    check('改派到另一台车', new_robot not in ('', first_robot), f'{first_robot} -> {new_robot}')

    done = wait_for(probe, lambda: probe.latest[task_id].state == 4, 150.0, '改派后完成')
    check('改派后任务完成', done, STATE_NAMES.get(probe.latest[task_id].state, '?'))
    proc.wait(timeout=60)


def scenario_d(probe):
    print('\n[D] 对位阶段目标丢失 -> 重试用尽 -> FAILED')
    prepare(probe)
    known = set(probe.latest)
    place_in_view(probe, 'block_green', ROBOT)

    task_id = wait_task(probe, 2, (3,), 90.0, '绿色目标进入对位', exclude=known)
    if task_id is None:
        check('绿色目标进入对位阶段', False, '未出现')
        return
    check('绿色目标进入对位阶段', True, task_id)

    proc = start_injector(['target-lost', '--block', 'block_green'])
    proc.wait(timeout=20)

    failed = wait_for(probe, lambda: probe.latest[task_id].state == 5, 180.0, '任务最终失败')
    check('任务最终转为 FAILED', failed, STATE_NAMES.get(probe.latest[task_id].state, '?'))
    check('恰好重试一次', probe.latest[task_id].retry_count == 1,
          f"retry={probe.latest[task_id].retry_count}")
    check('失败原因记录为 TARGET_LOST',
          probe.latest[task_id].failure_reason == 'TARGET_LOST',
          probe.latest[task_id].failure_reason)

    subprocess.run([sys.executable, INJECTOR, 'target-restore', '--block', 'block_green'],
                   check=False)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', default='ABCD')
    args = parser.parse_args()

    rclpy.init()
    probe = Probe()
    probe.spin_for(2.0)

    print('=' * 78)
    print('M7 故障注入验收')
    print('=' * 78)

    if 'A' in args.scenario:
        scenario_a(probe)
    if 'B' in args.scenario:
        scenario_b(probe)
    if 'D' in args.scenario:
        scenario_d(probe)
    # C 会打掉一台车的 Nav2，必须最后跑
    if 'C' in args.scenario:
        scenario_c(probe)

    # 收场：物块留在场外，避免验收结束后又自动开单，把场地留给下一轮
    clear_tasks(probe)
    idle_robots(probe)
    for block, (px, py) in PARK_FAR.items():
        set_pose(block, px, py)
    print('\n已清场，物块收至场外')

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
