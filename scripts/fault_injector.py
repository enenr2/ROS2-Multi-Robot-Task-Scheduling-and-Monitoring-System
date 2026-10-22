#!/usr/bin/env python3
"""故障注入器：在仿真里制造可复现故障，用于验证调度层的恢复能力。

    python3 scripts/fault_injector.py target-lost [--block block_blue]
    python3 scripts/fault_injector.py target-restore [--block block_blue]
    python3 scripts/fault_injector.py robot-offline --robot robot1 --hold 20
    python3 scripts/fault_injector.py nav-failure --robot robot1 --hold 25

注入手段与它们对应的故障语义：

| 命令            | 手段                                    | 调度层应看到                        |
|-----------------|-----------------------------------------|-------------------------------------|
| target-lost     | gz set_pose 把物块挪出视野               | agent 报 TARGET_LOST -> 重试/失败    |
| robot-offline   | SIGSTOP 该车 robot_agent（心跳停止）     | 心跳超时 -> 机器人离线，任务改派      |
| nav-failure     | SIGSTOP 该车 controller_server（导航卡死）| agent 报 NAV_ABORTED -> 取消并改派    |

用 SIGSTOP 而不是 kill：进程还在、内存还在，只是不再推进，比 kill 更接近
"卡死"，SIGCONT 能原样恢复。

nav-failure 会让该车的 Nav2 整栈下线：controller_server 停止发心跳后，
lifecycle_manager 的 bond 判定超时，会把全栈 deactivate。这不是副作用，
而是真实的"导航栈卡死需要人工恢复"——验收的是调度层能不能把任务改派给
另一台车并跑完，不要求故障车自愈。所以这个场景必须放在最后跑。

不要用 gz set_pose 搬动机器人模拟"原地打滑"：tb3 的 /robotN/odom 由轮速
积分，搬车不改里程计，Nav2 会认为自己在正常前进，反而永远不判失败。
"""
import argparse
import math
import subprocess
import sys
import time

WORLD = 'fleet'
BLOCK_Z = 0.1
PARK = {'block_red': (8.5, 7.5), 'block_green': (8.5, 9.0), 'block_blue': (8.5, 8.5)}
HOME = {'block_red': (2.5, 1.5), 'block_green': (4.0, -1.2), 'block_blue': (1.5, 3.0)}


def set_pose(name, x, y, z=BLOCK_Z, yaw=0.0):
    req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    for attempt in range(2):
        out = subprocess.run(
            ['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', req],
            capture_output=True, text=True, timeout=20)
        if 'data: true' in out.stdout:
            return
        if attempt == 0:
            time.sleep(0.5)
    raise RuntimeError(f'set_pose({name}) 失败: {out.stdout}{out.stderr}')


def find_pids(node_name, robot):
    """按"节点名 + 命名空间"定位进程。

    直接按可执行文件名匹配会误伤同名节点（两台车都有 controller_server），
    必须同时匹配 __node:= 与 __ns:=。
    """
    out = subprocess.run(['ps', '-eo', 'pid,args'], capture_output=True, text=True).stdout
    pids = []
    for line in out.splitlines()[1:]:
        pid, _, args = line.strip().partition(' ')
        if f'__node:={node_name}' in args and f'__ns:=/{robot}' in args:
            pids.append(int(pid))
    return pids


def freeze(pids, hold_s, label):
    if not pids:
        print(f'[EE] 没找到 {label} 进程，检查仿真是否在运行', file=sys.stderr)
        return False
    for pid in pids:
        subprocess.run(['kill', '-STOP', str(pid)], check=False)
    print(f'[..] 已冻结 {label}（pid {pids}），保持 {hold_s:.0f}s')
    try:
        time.sleep(hold_s)
    finally:
        for pid in pids:
            subprocess.run(['kill', '-CONT', str(pid)], check=False)
        print(f'[OK] 已恢复 {label}')
    return True


def main():
    parser = argparse.ArgumentParser(description='仿真故障注入器')
    sub = parser.add_subparsers(dest='command', required=True)

    p = sub.add_parser('target-lost', help='把物块移出视野')
    p.add_argument('--block', default='block_blue', choices=list(PARK))
    p.add_argument('--hold', type=float, default=0.0,
                   help='保持多少秒后自动放回原位（0 表示不放回）')

    p = sub.add_parser('target-restore', help='把物块放回初始位置')
    p.add_argument('--block', default='block_blue', choices=list(HOME))

    p = sub.add_parser('robot-offline', help='冻结某车的执行代理（心跳停止）')
    p.add_argument('--robot', required=True)
    p.add_argument('--hold', type=float, default=20.0)

    p = sub.add_parser('nav-failure', help='冻结某车的 controller_server（导航卡死）')
    p.add_argument('--robot', required=True)
    p.add_argument('--hold', type=float, default=25.0)

    args = parser.parse_args()

    if args.command == 'target-lost':
        x, y = PARK[args.block]
        set_pose(args.block, x, y)
        print(f'[OK] {args.block} 已移到 ({x}, {y})，离开所有相机视野')
        if args.hold > 0:
            time.sleep(args.hold)
            hx, hy = HOME[args.block]
            set_pose(args.block, hx, hy)
            print(f'[OK] {args.block} 已放回 ({hx}, {hy})')
        return 0

    if args.command == 'target-restore':
        x, y = HOME[args.block]
        set_pose(args.block, x, y)
        print(f'[OK] {args.block} 已放回 ({x}, {y})')
        return 0

    if args.command == 'robot-offline':
        return 0 if freeze(find_pids('robot_agent', args.robot), args.hold,
                            f'{args.robot} robot_agent') else 1

    if args.command == 'nav-failure':
        pids = find_pids('controller_server', args.robot)
        return 0 if freeze(pids, args.hold, f'{args.robot} controller_server') else 1

    return 1


if __name__ == '__main__':
    sys.exit(main())
