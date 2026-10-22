"""迟到回调与超时改派的时序测试（两个假 agent，不依赖 Gazebo）。

场景：
  1. robot1 离目标更近，先拿到任务；假 agent 配置成"永远不完成"
  2. manager 的 nav_timeout=2s 到期 → 取消 robot1 的 goal → 冷却 → 改派 robot2
  3. robot2 正常完成（故意把 task_status 推迟到 action result 之后发）
  4. robot1 迟到的取消结果 / 迟到的 task_status 不能把终态改乱

覆盖代码里"只信当前归属车的状态"和"迟到 action result 丢弃"两条保护。
"""
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from fleet_msgs.msg import RobotStatus, Task, TaskStatus
from fleet_msgs.srv import SubmitTask

TASKS = {}
STATUSES = {}
ROBOTS = {}
LOCK = threading.RLock()


def _on_task(msg):
    with LOCK:
        TASKS[msg.task_id] = msg


def _on_task_status(msg):
    with LOCK:
        STATUSES.setdefault(msg.task_id, []).append(msg)


def _on_robot_status(msg):
    with LOCK:
        ROBOTS[msg.robot_id] = msg


def generate_test_description():
    robot1 = launch_ros.actions.Node(
        package='fleet_integration',
        executable='fake_agent',
        namespace='robot1',
        parameters=[{'robot_id': 'robot1', 'hold_goals': True, 'pose_x': 5.0}],
        output='screen')

    robot2 = launch_ros.actions.Node(
        package='fleet_integration',
        executable='fake_agent',
        namespace='robot2',
        parameters=[{
            'robot_id': 'robot2',
            'hold_goals': False,
            'pose_x': 1.0,
            'status_after_result_ms': 300,   # result 先到，status 后到
        }],
        output='screen')

    manager = launch_ros.actions.Node(
        package='fleet_manager',
        executable='fleet_manager_node',
        name='fleet_manager',
        parameters=[{
            'robot_ids': ['robot1', 'robot2'],
            'dispatch_robot_ids': ['robot1', 'robot2'],
            'publish_markers': False,
            'nav_timeout': 2.0,
            'dock_timeout': 60.0,
            'retry_cooldown': 3.0,
        }],
        output='screen')

    return launch.LaunchDescription([
        robot1,
        robot2,
        manager,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class LateCallbackTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('late_callback_probe')
        cls.node.create_subscription(Task, '/fleet/tasks', _on_task, 20)
        cls.node.create_subscription(TaskStatus, '/robot1/task_status', _on_task_status, 50)
        cls.node.create_subscription(TaskStatus, '/robot2/task_status', _on_task_status, 50)
        cls.node.create_subscription(RobotStatus, '/robot1/robot_status', _on_robot_status, 20)
        cls.node.create_subscription(RobotStatus, '/robot2/robot_status', _on_robot_status, 20)
        cls.submit_client = cls.node.create_client(SubmitTask, '/fleet/submit_task')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_until(self, predicate, timeout, description):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with LOCK:
                if predicate():
                    return
            time.sleep(0.05)
        self.fail(f'等待超时（{timeout:.0f}s）：{description}')

    def call(self, client, request, timeout=15.0):
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: done.set())
        self.assertTrue(done.wait(timeout), '服务调用超时')
        return future.result()

    def snapshot(self, task_id):
        with LOCK:
            return TASKS.get(task_id)

    def test_timeout_reassign_ignores_late_result(self):
        self.wait_until(
            lambda: ROBOTS.get('robot1') is not None and ROBOTS.get('robot1').heartbeat_seq >= 2
            and ROBOTS.get('robot2') is not None and ROBOTS.get('robot2').heartbeat_seq >= 2,
            20.0, '两个假 agent 心跳上线')

        self.assertTrue(self.submit_client.wait_for_service(timeout_sec=15.0))
        request = SubmitTask.Request()
        request.target_color = 3
        request.target_pose.position.x = 200.0
        request.target_pose.orientation.w = 1.0
        response = self.call(self.submit_client, request)
        self.assertTrue(response.accepted, response.message)
        task_id = response.task_id

        # robot1 更近，先拿到；随后被看门狗超时收走
        self.wait_until(
            lambda: self.snapshot(task_id) is not None
            and self.snapshot(task_id).assigned_robot_id == 'robot1',
            10.0, '任务先派给 robot1')
        self.wait_until(
            lambda: self.snapshot(task_id) is not None
            and self.snapshot(task_id).assigned_robot_id == 'robot2',
            20.0, '超时后改派 robot2')

        # robot2 完成（result 先到、status 后到，调度端必须自己收尾）
        self.wait_until(lambda: self.snapshot(task_id).state == TaskStatus.STATE_DONE,
                        20.0, '任务最终 DONE')
        self.assertEqual(self.snapshot(task_id).assigned_robot_id, 'robot2')

        # 等 robot1 的取消结果与迟到的 status 都到达，终态不能被改乱
        time.sleep(1.5)
        self.assertEqual(self.snapshot(task_id).state, TaskStatus.STATE_DONE,
                         '迟到的取消/结果不能把已完成任务改回失败')

        with LOCK:
            robot1_states = [msg.state for msg in STATUSES.get(task_id, [])
                             if msg.robot_id == 'robot1']
        self.assertNotIn(TaskStatus.STATE_FAILED, robot1_states,
                         '被取消的旧车不应上报 FAILED')
        self.assertNotIn(TaskStatus.STATE_DONE, robot1_states,
                         '旧车不应上报完成')


if __name__ == '__main__':
    unittest.main()