#!/usr/bin/env python3
"""M4 感知层验收脚本：测距标定 + 三色识别 + 误检率 + 丢失链路。

三项验收标准（与里程碑 M4 对齐）：
  1. 测距误差：1/2/3 m 静态目标，|实测 - 几何真值| <= 0.2 m
  2. 三色识别：三种物块放在同一位置，检出的颜色必须与物块颜色一致
  3. 误检率：视野内无任何物块时，连续 60 s 上报的检测数应为 0
  4. 丢失链路：把已捕获的目标移出视野，应收到恰好 1 条 lost 事件

真值从哪来：物块位置由本脚本用 gz set_pose 服务设定（它就是真值），
相机位姿从 TF 取。测距的理论值必须用**目标在光轴方向上的深度**
Z = (P - C)·o（o 为光轴单位向量），而不是"相机到目标的两点欧氏距离"——
后者在相机有俯角时会系统性偏大，用它当基准会白白放过测距误差。

用法：
  source install/setup.bash
  python3 src/fleet_perception/scripts/verify_perception.py
"""
import math
import statistics
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

from fleet_msgs.msg import Detection

WORLD = 'fleet'
ROBOT = 'robot1'
CAMERA_FRAME = ROBOT + '_camera_link'
BLOCK_SIZE = 0.2
BLOCK_Z = 0.1          # 物块中心高度
DISTANCES = [1.0, 2.0, 3.0]
COLORS = {'block_red': (Detection.COLOR_RED, 'RED'),
          'block_green': (Detection.COLOR_GREEN, 'GREEN'),
          'block_blue': (Detection.COLOR_BLUE, 'BLUE')}
HOMES = {'block_red': (2.5, 1.5), 'block_green': (4.0, -1.2), 'block_blue': (1.5, 3.0)}

FAILURES = []


def check(label, ok, detail=''):
    mark = '通过' if ok else '不通过'
    print(f'  [{mark}] {label}{(" — " + detail) if detail else ""}')
    if not ok:
        FAILURES.append(label)


def quat_to_matrix(qx, qy, qz, qw):
    return [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def set_pose(name, x, y, z=BLOCK_Z, yaw=0.0):
    req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    out = subprocess.run(
        ['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '2000', '--req', req],
        capture_output=True, text=True, timeout=15)
    if 'data: true' not in out.stdout:
        raise RuntimeError(f'set_pose({name}) 失败: {out.stdout} {out.stderr}')


def read_pose(name):
    """回读 gz 里的真实位姿：set_pose 返回 true 不等于物块已在目标位置（
    被墙/机器人挤开时会被物理引擎推走），验收必须用回读值当真值。"""
    out = subprocess.run(['gz', 'model', '-m', name, '-p'],
                         capture_output=True, text=True, timeout=15)
    for line in out.stdout.splitlines():
        line = line.strip()
        if line.startswith('[') and ']' in line:
            vals = line.strip('[]').split()
            if len(vals) >= 2:
                return float(vals[0]), float(vals[1])
    raise RuntimeError(f'无法解析 {name} 的位姿: {out.stdout}{out.stderr}')


FX = 277.191   # 来自 gz camera_info，与节点日志一致
CX, CY = 160.0, 120.0


def project(cam_pos, rot, point):
    """世界点 -> 像素坐标。相机系约定：x 前、y 左、z 上，故 u 随 y 反向。"""
    d = sub(point, cam_pos)
    x = dot(rot[0], d)
    y = dot(rot[1], d)
    z = dot(rot[2], d)
    if x <= 1e-6:
        return None
    return CX - FX * y / x, CY - FX * z / x


def theory(cam_pos, rot, center, yaw):
    """理论基准：把物块正面的四个角投到像平面。

    返回 (理论框宽, 理论外接框中心, 正面中心深度, 正面底边深度)。
    "理论框宽"是本算法（bbox 宽度反推距离）的直接基准；两个深度分别用于
    说明几何真值——底边深度才是与像素宽度对应的那一行的真实深度。
    """
    fh = (math.cos(yaw), math.sin(yaw), 0.0)
    tangent = (-fh[1], fh[0], 0.0)
    face = (center[0] - BLOCK_SIZE / 2 * fh[0], center[1] - BLOCK_SIZE / 2 * fh[1], center[2])
    us, vs = [], []
    for a in (-BLOCK_SIZE / 2, BLOCK_SIZE / 2):
        for z in (0.0, BLOCK_SIZE):
            p = (face[0] + a * tangent[0], face[1] + a * tangent[1], z)
            uv = project(cam_pos, rot, p)
            if uv is None:
                return None
            us.append(uv[0])
            vs.append(uv[1])
    width = max(us) - min(us)
    z_center = dot(sub(face, cam_pos), rot[0])
    z_bottom = dot(sub((face[0], face[1], 0.0), cam_pos), rot[0])
    return width, ((min(us) + max(us)) / 2, (min(vs) + max(vs)) / 2), z_center, z_bottom


class Probe(Node):
    def __init__(self):
        super().__init__('perception_verify', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.dets = []
        self.lost = 0
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Detection, f'/{ROBOT}/detections', self.on_det, qos)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def on_det(self, msg):
        if msg.lost:
            self.lost += 1
            return
        self.dets.append(msg)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def camera_pose(self):
        last = None
        for _ in range(60):
            try:
                tf = self.buffer.lookup_transform('map', CAMERA_FRAME, rclpy.time.Time())
                break
            except Exception as ex:
                last = ex
                self.spin_for(0.3)
        else:
            raise RuntimeError(f'TF map -> {CAMERA_FRAME} 不可用: {last}')
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), quat_to_matrix(r.x, r.y, r.z, r.w)


def sample(probe, dwell):
    probe.dets.clear()
    t0 = time.time()
    probe.spin_for(dwell)
    return probe.dets, time.time() - t0


def main():
    rclpy.init()
    probe = Probe()
    probe.spin_for(2.0)

    pubs = probe.count_publishers(f'/{ROBOT}/detections')
    if pubs != 1:
        print(f'检测话题 /{ROBOT}/detections 上有 {pubs} 个发布者，预期恰好 1 个。')
        print('请先关掉手动启动的多余感知节点，否则样本会混在一起。')
        return 2

    cam_pos, rot = probe.camera_pose()
    forward = (rot[0][0], rot[1][0], rot[2][0])
    horiz = math.hypot(forward[0], forward[1])
    fh = (forward[0] / horiz, forward[1] / horiz, 0.0)
    yaw_cam = math.atan2(fh[1], fh[0])
    pitch_deg = math.degrees(math.asin(-forward[2]))

    print('=' * 78)
    print('M4 感知层验收')
    print('=' * 78)
    print(f'相机: pos=({cam_pos[0]:.3f}, {cam_pos[1]:.3f}, {cam_pos[2]:.3f}) '
          f'俯角={pitch_deg:.1f}° 世界偏航={math.degrees(math.atan2(fh[1], fh[0])):.1f}°')
    print(f'物块: {BLOCK_SIZE} m 立方体，中心高度 {BLOCK_Z} m\n')

    # ---------- 1. 测距标定 ----------
    print('[1] 单目测距标定（红块放在相机正前方，物块正面朝向相机）')
    hdr = (f'{"水平距离":>8} {"像素宽":>7} {"理论宽":>7} {"实测距离":>9} {"底边深度":>9} '
           f'{"误差":>8} {"偏角":>8} {"world误差":>10} {"帧数":>5} {"上报率":>8}')
    print(hdr)
    print('-' * len(hdr))
    max_err = 0.0
    for d in DISTANCES:
        # 先清场：其余两个物块挪出视野，避免它们混进红块样本
        for other in HOMES:
            if other != 'block_red':
                set_pose(other, 8.5, 8.5)
        set_pose('block_red', cam_pos[0] + d * fh[0], cam_pos[1] + d * fh[1], yaw=yaw_cam)
        probe.spin_for(3.0)                 # 等去抖器稳定
        bx, by = read_pose('block_red')     # 以回读的真值为准
        dets, elapsed = sample(probe, 6.0)

        t = theory(cam_pos, rot, (bx, by, BLOCK_Z), yaw_cam)
        if t is None or not dets:
            print(f'{d:>8.1f}  未检出或几何不可解')
            check(f'{d:.0f} m 处能检出目标', False)
            continue
        pred_w, _, z_center, z_bottom = t

        r_med = statistics.median([s.range for s in dets])
        w_med = statistics.median([s.bbox.width for s in dets])
        b_med = math.degrees(statistics.median([s.bearing for s in dets]))
        err = r_med - z_bottom
        wp_err = statistics.median(
            [math.dist((s.world_pose.x, s.world_pose.y, s.world_pose.z),
                       (bx - BLOCK_SIZE / 2 * fh[0], by - BLOCK_SIZE / 2 * fh[1], BLOCK_Z))
             for s in dets])
        max_err = max(max_err, abs(err))

        print(f'{d:>8.1f} {w_med:>7.0f} {pred_w:>7.1f} {r_med:>9.3f} {z_bottom:>9.3f} '
              f'{err:>+8.3f} {b_med:>+7.2f}° {wp_err:>9.3f}m {len(dets):>5} '
              f'{len(dets)/elapsed:>7.1f}Hz')

    check('正前方 1~3 m 测距误差 <= 0.2 m', max_err <= 0.2, f'最大 {max_err:.3f} m')

    # ---------- 1b. 偏轴退化 ----------
    print('\n[1b] 偏轴测距退化（2.0 m，物块正面朝向相机，逐个角度转过去）')
    hdr2 = (f'{"偏轴角":>8} {"像素宽":>7} {"理论宽":>7} {"实测距离":>9} {"底边深度":>9} '
            f'{"误差":>8} {"实测偏角":>9} {"理论偏角":>9}')
    print(hdr2)
    print('-' * len(hdr2))
    for angle_deg in (-24.0, -12.0, 0.0, 12.0, 24.0):
        th = math.radians(angle_deg)
        dx = fh[0] * math.cos(th) - fh[1] * math.sin(th)
        dy = fh[0] * math.sin(th) + fh[1] * math.cos(th)
        bx = cam_pos[0] + 2.0 * dx
        by = cam_pos[1] + 2.0 * dy
        set_pose('block_red', bx, by, yaw=math.atan2(dy, dx))
        probe.spin_for(3.0)
        bx, by = read_pose('block_red')
        dets, _ = sample(probe, 6.0)

        t = theory(cam_pos, rot, (bx, by, BLOCK_Z), math.atan2(dy, dx))
        if t is None or not dets:
            print(f'{angle_deg:>8.0f}  未检出')
            continue
        pred_w, uv_center, _, z_bottom = t
        exp_bearing = math.degrees(math.atan((uv_center[0] - CX) / FX))
        r_med = statistics.median([s.range for s in dets])
        w_med = statistics.median([s.bbox.width for s in dets])
        b_med = math.degrees(statistics.median([s.bearing for s in dets]))

        print(f'{angle_deg:>+8.0f}° {w_med:>7.0f} {pred_w:>7.1f} {r_med:>9.3f} '
              f'{z_bottom:>9.3f} {r_med - z_bottom:>+8.3f} {b_med:>+8.2f}° '
              f'{exp_bearing:>+8.2f}°')
    print('  说明：偏轴越远，外接框会开始装进物块的侧面投影，'
          '像素宽度虚高 -> 测距系统性偏小。')

    # ---------- 2. 三色识别 ----------
    print('\n[2] 三色识别（每种物块放到同一位置，检查报出的颜色）')
    spot = (cam_pos[0] + 2.0 * fh[0], cam_pos[1] + 2.0 * fh[1])
    for block, (want, name) in COLORS.items():
        # 其余物块必须移出视野，否则"绿色被检出"会被误记成红块的错判
        for other in COLORS:
            if other != block:
                set_pose(other, 8.5, 8.5)
        set_pose(block, spot[0], spot[1], yaw=yaw_cam)
        probe.spin_for(3.5)
        dets, _ = sample(probe, 6.0)

        got = {}
        for s in dets:
            label = {1: 'RED', 2: 'GREEN', 3: 'BLUE'}.get(s.color, '?')
            got[label] = got.get(label, 0) + 1
        right = got.get(name, 0)
        wrong = sum(v for k, v in got.items() if k != name)
        r_med = statistics.median([s.range for s in dets]) if dets else float('nan')

        print(f'  {block:<12} 期望 {name:<6} 实得 {got} 中位距离 {r_med:.3f} m')
        check(f'{block} 颜色判定正确', right > 0 and wrong == 0,
              f'正确 {right} 帧 / 错误 {wrong} 帧')
        set_pose(block, *HOMES[block])

    # ---------- 3. 误检率 ----------
    print('\n[3] 误检率（三个物块全部移到视野外，观察 60 s）')
    park = {'block_red': (8.5, 8.5), 'block_green': (8.5, 7.5), 'block_blue': (8.5, 9.5)}
    for block, (x, y) in park.items():
        set_pose(block, x, y)
    probe.spin_for(4.0)                     # 等去抖器进入 lost，避免把残留算成误检
    probe.dets.clear()
    dets, elapsed = sample(probe, 60.0)
    print(f'  60 s 内上报检测 {len(dets)} 条（目标已全部移出视野）')
    check('视野内无目标时零误检', len(dets) == 0)

    # ---------- 4. 丢失链路 ----------
    print('\n[4] 丢失事件链路')
    for block, (x, y) in HOMES.items():
        if block != 'block_red':
            set_pose(block, x, y)
    set_pose('block_red', spot[0], spot[1], yaw=yaw_cam)
    probe.spin_for(4.0)
    before = probe.lost
    set_pose('block_red', 8.5, 9.0)         # 移出视野
    probe.spin_for(12.0)
    got = probe.lost - before
    print(f'  移走已捕获目标后收到丢失事件 {got} 条')
    check('丢失事件恰好上报一次', got == 1, f'实得 {got} 条')

    for block, (x, y) in HOMES.items():
        set_pose(block, x, y)
    print('\n已恢复所有物块到初始位置')

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