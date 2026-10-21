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

    world_file = os.path.join(bringup, 'worlds', 'fleet_empty.sdf')
    bridge_yaml = os.path.join(bringup, 'config', 'bridge.yaml')
    models_dir = os.path.join(bringup, 'models')
    tb3_models = os.path.join(tb3_gz, 'models')
    urdf_file = os.path.join(tb3_gz, 'urdf', 'turtlebot3_burger.urdf')
    rviz_config = os.path.join(bringup, 'config', 'fleet.rviz')

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

        # 世界系 → 各车 odom 系的静态变换：
        # gz 的里程计以出生点为原点，所以用出生位姿当静态变换，
        # 两台车才能同时显示在同一个 world 坐标系下
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name + '_world_tf',
            arguments=['--x', x, '--y', y, '--z', '0',
                       '--frame-id', 'world',
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

    # ⑥ 集群可视化：RViz2 统一在 world 坐标系下显示两台车
    # 延迟 6 秒启动：等 clock/TF/robot_description 就绪后再建视图，
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
