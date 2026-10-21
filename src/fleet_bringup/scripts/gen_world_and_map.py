#!/usr/bin/env python3
"""生成仿真世界与配套栅格地图。

单一几何来源：墙壁与物块的坐标只在这里写一次，同时产出
  1. worlds/fleet_tasks.sdf      —— Gazebo 世界
  2. maps/fleet_map.pgm/.yaml    —— Nav2 map_server 用的静态地图

这样地图与世界的几何天然一致，不需要跑 SLAM 建图。
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

# ---------------------------------------------------------------- 几何定义

FIELD_HALF = 10.0          # 场地半径：-10 ~ +10 米
WALL_T = 0.2               # 墙厚
WALL_H = 0.8               # 墙高（高于激光雷达 0.172m，低于相机视场外）
BLOCK_SIZE = 0.2           # 物块边长

# 墙壁：(中心x, 中心y, x方向长度, y方向长度)
# 4 面外墙 + 3 段内隔断，内隔断有意造出走廊与死角，供导航失败注入使用
WALLS = [
    (0.0, -FIELD_HALF, 2 * FIELD_HALF + WALL_T, WALL_T),   # 南
    (0.0,  FIELD_HALF, 2 * FIELD_HALF + WALL_T, WALL_T),   # 北
    (-FIELD_HALF, 0.0, WALL_T, 2 * FIELD_HALF + WALL_T),   # 西
    ( FIELD_HALF, 0.0, WALL_T, 2 * FIELD_HALF + WALL_T),   # 东
    (0.0, 5.0, WALL_T, 6.0),                               # 内隔断 1：竖向 y∈[2,8]
    (-4.0, 0.0, 6.0, WALL_T),                              # 内隔断 2：横向 x∈[-7,-1]
    (5.0, -3.0, WALL_T, 5.0),                              # 内隔断 3：竖向 y∈[-5.5,-0.5]
]

# 任务物块：(名称, x, y, 颜色名, RGB 漫反射)
BLOCKS = [
    ("block_red",   2.5,  1.5, "red",   (0.85, 0.05, 0.05)),
    ("block_green", 4.0, -1.2, "green", (0.05, 0.85, 0.05)),
    ("block_blue",  1.5,  3.0, "blue",  (0.05, 0.05, 0.85)),
]

# 地图参数
MAP_RES = 0.05             # 米/像素
MAP_W = int(2 * FIELD_HALF / MAP_RES)   # 400
MAP_H = int(2 * FIELD_HALF / MAP_RES)   # 400


# ---------------------------------------------------------------- SDF 生成

def _box(name, cx, cy, sx, sy, h, z, rgb, rgba=None, static=True):
    r, g, b = rgb
    return f"""    <model name="{name}">
      <static>{'true' if static else 'false'}</static>
      <pose>{cx} {cy} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} 1</ambient>
            <diffuse>{r} {g} {b} 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def gen_world():
    models = []

    # 地面：自建平面而非燃料源模型，去掉网络依赖；浅灰低饱和，不干扰颜色识别
    models.append(_box("ground", 0, 0, 30, 30, 0.1, -0.05, (0.75, 0.75, 0.75)))

    for i, (cx, cy, sx, sy) in enumerate(WALLS):
        models.append(_box(f"wall_{i}", cx, cy, sx, sy, WALL_H, WALL_H / 2,
                           (0.5, 0.5, 0.55)))

    for name, x, y, _color, rgb in BLOCKS:
        models.append(_box(name, x, y, BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE,
                           BLOCK_SIZE / 2, rgb))

    return f"""<?xml version="1.0"?>
<!-- 本文件由 scripts/gen_world_and_map.py 生成，请勿手改 -->
<sdf version="1.6">
  <world name="fleet">
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>

    <!-- 平行光：自建而非燃料源 Sun，去掉网络依赖 -->
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <scene>
      <shadows>false</shadows>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.3 0.3 0.35 1</background>
    </scene>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-3 -3 14 0 1.2 0.7</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <physics type="ode">
      <real_time_update_rate>250.0</real_time_update_rate>
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver><type>quick</type><iters>150</iters></solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.0</contact_max_correcting_vel>
          <contact_surface_layer>0.01</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

{''.join(models)}  </world>
</sdf>
"""


# ---------------------------------------------------------------- 地图生成

def gen_map():
    # 255 = 可通行，0 = 障碍
    grid = [[255] * MAP_W for _ in range(MAP_H)]

    def fill(cx, cy, sx, sy):
        """把世界坐标的矩形填成障碍像素。"""
        x0, x1 = cx - sx / 2, cx + sx / 2
        y0, y1 = cy - sy / 2, cy + sy / 2
        # 世界 -> 像素：列随 x 增大，行随 y 减小（PGM 第 0 行在图像顶部）
        c0 = int(math.floor((x0 + FIELD_HALF) / MAP_RES))
        c1 = int(math.ceil((x1 + FIELD_HALF) / MAP_RES)) - 1
        r0 = int(math.floor((FIELD_HALF - y1) / MAP_RES))
        r1 = int(math.ceil((FIELD_HALF - y0) / MAP_RES)) - 1
        for r in range(max(0, r0), min(MAP_H - 1, r1) + 1):
            for c in range(max(0, c0), min(MAP_W - 1, c1) + 1):
                grid[r][c] = 0

    for cx, cy, sx, sy in WALLS:
        fill(cx, cy, sx, sy)
    # 物块也标记为障碍：机器人应停在 standoff 处，不能压上去
    for _name, x, y, _c, _rgb in BLOCKS:
        fill(x, y, BLOCK_SIZE, BLOCK_SIZE)

    occupied = sum(1 for r in grid for v in r if v == 0)

    header = (f"P5\n# fleet_map {MAP_W}x{MAP_H} @ {MAP_RES} m/px\n"
              f"{MAP_W} {MAP_H}\n255\n")
    body = bytearray()
    for row in grid:
        body.extend(row)
    return header.encode("ascii") + bytes(body), occupied


def gen_map_yaml():
    return f"""image: fleet_map.pgm
mode: trinary
resolution: {MAP_RES}
origin: [{-FIELD_HALF}, {-FIELD_HALF}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""


# ---------------------------------------------------------------- 主流程

def main():
    world_path = os.path.join(PKG, "worlds", "fleet_tasks.sdf")
    maps_dir = os.path.join(PKG, "maps")
    os.makedirs(maps_dir, exist_ok=True)

    with open(world_path, "w") as f:
        f.write(gen_world())
    print(f"已生成 {world_path}")

    pgm = os.path.join(maps_dir, "fleet_map.pgm")
    data, occupied = gen_map()
    with open(pgm, "wb") as f:
        f.write(data)
    print(f"已生成 {pgm}")

    yml = os.path.join(maps_dir, "fleet_map.yaml")
    with open(yml, "w") as f:
        f.write(gen_map_yaml())
    print(f"已生成 {yml}")

    print(f"地图尺寸 {MAP_W}x{MAP_H}，障碍像素 {occupied}"
          f"（占 {occupied / (MAP_W * MAP_H) * 100:.1f}%）")


if __name__ == "__main__":
    main()
