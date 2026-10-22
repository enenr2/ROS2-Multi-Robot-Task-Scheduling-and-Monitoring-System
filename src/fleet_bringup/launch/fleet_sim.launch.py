"""多机器人仿真启动：一个 gz 世界 + 两台 namespace 隔离的 TurtleBot3。"""
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    bringup = get_package_share_directory('fleet_bringup')
    tb3_gz = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(bringup, 'worlds', 'fleet_tasks.sdf')
    bridge_yaml = os.path.join(bringup, 'config', 'bridge.yaml')
    models_dir = os.path.join(bringup, 'models')
    tb3_models = os.path.join(tb3_gz, 'models')
    urdf_file = os.path.join(bringup, 'urdf', 'turtlebot3_burger_fleet.urdf')
    rviz_config = os.path.join(bringup, 'config', 'fleet.rviz')
    map_yaml = os.path.join(bringup, 'maps', 'fleet_map.yaml')
    nav2_params = os.path.join(bringup, 'config', 'nav2_robot1.yaml')

    with open(urdf_file, 'r') as f:
        urdf_text = f.read()

    def prefixed_description(text, name):
        """把 URDF 里所有 link/joint 名字加上机器人前缀。

        只靠 RSP 的 frame_prefix 时，两份 robot_description 内容完全一样，
        RViz 里两个 RobotModel 会互相顶掉、只渲染一台车；所以这里直接
        生成两份独立的 URDF，让连杆名和 TF 帧名一致。
        """
        root = ET.fromstring(text)
        root.set('name', name)
        for el in root.iter():
            if el.tag == 'link':
                el.set('name', name + '_' + el.get('name'))
            elif el.tag == 'joint':
                el.set('name', name + '_' + el.get('name'))
                # 关节内 parent/child 的 link 引用也要同步改名
                for ref in ('parent', 'child'):
                    node = el.find(ref)
                    if node is not None:
                        node.set('link', name + '_' + node.get('link'))
        return ET.tostring(root, encoding='unicode')

    # 机器人清单：名字、出生点（两车错开，避免初始碰撞）
    robots = [
        ('robot1', '0.0', '0.0'),
        ('robot2', '1.5', '0.0'),
    ]

    actions = [
        # 模型资源路径：我们的两台车 + TB3 公共 mesh 资源
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_dir + ':' + tb3_models),
        # 全局仿真时间：所有节点统一用 /clock
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument('gui', default_value='false',
                              description='是否启动 gz GUI（VM 上建议 false）'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否启动 RViz2 集群可视化'),
        DeclareLaunchArgument('nav2', default_value='true',
                              description='是否为 robot1 启动 Nav2 导航栈'),
        DeclareLaunchArgument('perception', default_value='true',
                              description='是否为每台车启动视觉感知节点'),
        DeclareLaunchArgument('agent', default_value='true',
                              description='是否为每台车启动执行代理（ExecuteTask action）'),
        DeclareLaunchArgument('manager', default_value='true',
                              description='是否启动调度中心（任务分配与监控）'),

        # ① 物理服务端（-r 加载即运行）
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '-v2', world_file],
                       output='screen'),
        # ② 可选 GUI
        ExecuteProcess(cmd=['gz', 'sim', '-g', '-v2'],
                       condition=IfCondition(LaunchConfiguration('gui')),
                       output='screen'),
        # ③ 桥：一张 yaml 管两台车的全部话题
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': bridge_yaml}],
             output='screen'),
        # ③' 静态地图：Nav2 的 costmap 与全局规划都以它为基准。
        # map_server 是生命周期节点，靠 lifecycle_manager 自动 configure+activate
        Node(package='nav2_map_server', executable='map_server',
             name='fleet_map_server',
             parameters=[{'yaml_filename': map_yaml}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map',
             parameters=[{'autostart': True,
                          'node_names': ['fleet_map_server']}],
             output='screen'),
    ]

    for name, x, y in robots:
        # ④ 每台车一份 robot_state_publisher（frame_prefix 隔离坐标树）
        actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=name + '_state_publisher',
            parameters=[{
                'robot_description': prefixed_description(urdf_text, name),
            }],
            # joint_states 走各自命名空间；robot_description 也分开发布，
            # 否则两台车的 RViz RobotModel 会抢同一个话题
            remappings=[('joint_states', name + '/joint_states'),
                        ('robot_description', name + '/robot_description')],
            output='screen'))

        # map → 各车 odom 系的静态变换：
        # gz 的里程计以出生点为原点，所以用出生位姿当静态变换，
        # 两台车才能同时显示在同一个地图坐标系下（帧名遵循 Nav2 惯例）
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name + '_map_tf',
            arguments=['--x', x, '--y', y, '--z', '0',
                       '--frame-id', 'map',
                       '--child-frame-id', name + '_odom'],
            output='screen'))

        # ⑤ 生成实体
        actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-world', 'fleet',
                       '-file', os.path.join(models_dir, name, 'model.sdf'),
                       '-name', name,
                       '-x', x, '-y', y, '-z', '0.05'],
            output='screen'))

    # ⑥ Nav2 导航栈：只给 robot1 起（M2 阶段先跑通单机）
    #
    # 三处必须显式处理，否则多机 TF 场景下必挂：
    #   1. tf / tf_static 强制指向全局话题。Nav2 官方 launch 把 '/tf' 重映射成
    #      相对名 'tf'，在命名空间下会变成 /robot1/tf —— 但我们两台车共用
    #      一棵 TF 树（帧名已带前缀），必须都读全局 /tf
    #   2. map 指向全局地图。全局代价地图的静态层默认订阅相对 'map'，
    #      在命名空间下会解析成 /robot1/map，而地图是全局共享的 /map
    #   3. 帧名在 nav2_robot1.yaml 里已写成 robot1_xxx，与 TF 树一致
    NAV2_NS = 'robot1'

    def nav2_node(pkg, exe, node_name, extra_remaps=(), extra_params=None):
        return Node(
            package=pkg, executable=exe, name=node_name,
            namespace=NAV2_NS,
            parameters=[nav2_params] + ([extra_params] if extra_params else []),
            remappings=[('/tf', '/tf'), ('/tf_static', '/tf_static')] + list(extra_remaps),
            condition=IfCondition(LaunchConfiguration('nav2')),
            output='screen')

    nav2_nodes = [
        # 代价地图是 controller_server / planner_server 的子节点，
        # 所以 map 的重映射挂在父节点上，子节点同进程生效。
        # 注意必须写全限定名 '/robot1/map'：Nav2 把 map 解析成父命名空间下的
        # /robot1/map，用相对名 'map' 作重映射源匹配不上（实测无效）
        nav2_node('nav2_controller', 'controller_server', 'controller_server'),
        nav2_node('nav2_planner', 'planner_server', 'planner_server',
                  [('/robot1/map', '/map')]),
        nav2_node('nav2_behaviors', 'behavior_server', 'behavior_server'),
        nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator'),
        # service_timeout 默认 5s：本机在 RViz + 双相机同时渲染时会超时，
        # 表现为 get_state 请求发不出去、bringup 直接中止（实测遇到过两次）。
        # 放宽到 20s 换启动成功率，代价是真挂时多等一会儿
        nav2_node('nav2_lifecycle_manager', 'lifecycle_manager',
                  'lifecycle_manager_navigation', (),
                  {'service_timeout': 20.0}),
    ]

    # 延迟启动 Nav2：等 gz 出 clock、桥出 odom/scan、TF 树建起来之后再拉，
    # 否则生命周期节点会因拿不到初始位姿而卡在 inactive
    actions.append(TimerAction(period=10.0, actions=nav2_nodes))

    # ⑥' 视觉感知：每台车一个节点，订阅各自命名空间下的相机，输出 detections
    # 感知节点不依赖 Nav2，但依赖 TF 树（map <- robotN_camera_link），
    # 所以稍等几秒再拉起，避免启动初期 TF 查询失败刷告警
    perception_nodes = []
    for name, _x, _y in robots:
        perception_nodes.append(Node(
            package='fleet_perception',
            executable='perception_node',
            name='perception_node',
            namespace=name,
            parameters=[{
                'robot_id': name,
                'world_frame': 'map',
                'target_width': 0.2,
                'max_range': 5.0,
                'publish_debug_image': False,
            }],
            condition=IfCondition(LaunchConfiguration('perception')),
            output='screen'))
    actions.append(TimerAction(period=6.0, actions=perception_nodes))

    # ⑥'' 执行代理：每台车一个，对外提供 /robotN/execute_task action。
    # 比 Nav2 晚一点起，避免首个任务在 Nav2 服务端就绪前就发进来
    agent_nodes = []
    for name, _x, _y in robots:
        agent_nodes.append(Node(
            package='fleet_agent',
            executable='robot_agent',
            name='robot_agent',
            namespace=name,
            parameters=[{
                'robot_id': name,
                'world_frame': 'map',
                'standoff': 0.6,
                'pre_dock_margin': 0.8,
                'nav_timeout': 90.0,
                'dock_timeout': 20.0,
            }],
            condition=IfCondition(LaunchConfiguration('agent')),
            output='screen'))
    actions.append(TimerAction(period=14.0, actions=agent_nodes))

    # ⑥''' 调度中心：订阅各车感知与状态，按任务表分配并直调 execute_task。
    # 放在代理之后起，保证第一拍分配时 action 服务端已就绪
    actions.append(TimerAction(
        period=18.0,
        actions=[Node(
            package='fleet_manager',
            executable='fleet_manager_node',
            name='fleet_manager',
            parameters=[{
                'robot_ids': [name for name, _x, _y in robots],
                # 目前只有 robot1 接了 Nav2，robot2 只监控不派活
                'dispatch_robot_ids': ['robot1'],
                'standoff': 0.6,
                'publish_markers': True,
            }],
            condition=IfCondition(LaunchConfiguration('manager')),
            output='screen')]))

    # ⑦ 集群可视化：RViz2 统一在 map 坐标系下显示两台车
    # 延迟启动：等 clock/TF/robot_description 就绪后再建视图，
    # 否则 RViz 会在 TF 缺席时初始化相机，视角会歪掉
    actions.append(TimerAction(
        period=6.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='fleet_rviz',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen')]))

    return LaunchDescription(actions)
