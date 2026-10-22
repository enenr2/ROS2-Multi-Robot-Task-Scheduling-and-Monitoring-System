"""真执行代理 + 假 Nav2：导航中 / 对位中 / 搜索中取消的 action+topic 一致性。

校验：
  * action result 是 CANCELED（不是 ABORTED/SUCCEEDED）
  * TaskStatus 终态是 CANCELED（6），且没有 FAILED（5）
  * RobotStatus.tasks_canceled +1、tasks_failed 不变
  * 三个阶段的取消 detail 分别是 导航中/对位中/搜索中
"""
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from fleet_msgs.action import ExecuteTask
from fleet_msgs.msg import RobotStatus, TaskStatus
from nav_msgs.msg import Odometry

STATUSES = {}
ROBOTS = {}
LOCK = threading.RLock()

STATE_NAVIGATING = 2
STATE_DOCKING = 3


def _on_status(msg):
    with LOCK:
        STATUSES.setdefault(msg.task_id, []).append(msg)


def _on_robot(msg):
    with LOCK:
        ROBOTS['r'] = msg


def generate_test_description():
    fake_nav = launch_ros.actions.Node(
        package='fleet_integration',
        executable='fake_nav',
        namespace='robot1',
        parameters=[{'delay_s': 2.0}],
        output='screen')

    agent = launch_ros.actions.Node(
        package='fleet_agent',
        executable='robot_agent',
        namespace='robot1',
        parameters=[{
            'robot_id': 'robot1',
            'world_frame': 'map',
            'standoff': 0.6,
            'nav_timeout': 20.0,
            'dock_timeout': 10.0,
        }],
        output='screen')

    return launch.LaunchDescription([
        fake_nav,
        agent,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class AgentCancelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('agent_cancel_probe')
        cls.node.create_subscription(TaskStatus, '/robot1/task_status', _on_status, 50)
        cls.node.create_subscription(RobotStatus, '/robot1/robot_status', _on_robot, 20)

        # 真 agent 需要 odom 才肯开始任务
        cls.odom_pub = cls.node.create_publisher(Odometry, '/robot1/odom', 10)
        cls.node.create_timer(0.05, cls._publish_odom)

        cls.client = ActionClient(cls.node, ExecuteTask, '/robot1/execute_task')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        threading.Thread(target=cls.executor.spin, daemon=True).start()
        assert cls.client.wait_for_server(timeout_sec=30.0), 'execute_task 不可用'

    @classmethod
    def _publish_odom(cls):
        msg = Odometry()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.orientation.w = 1.0
        cls.odom_pub.publish(msg)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.try_shutdown()

    # ---------------- 工具 ----------------

    def wait_until(self, predicate, timeout, description):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with LOCK:
                if predicate():
                    return
            time.sleep(0.05)
        self.fail(f'等待超时（{timeout:.0f}s）：{description}')

    def wait_future(self, future, timeout, description):
        event = threading.Event()
        future.add_done_callback(lambda _f: event.set())
        self.assertTrue(event.wait(timeout), f'{description} 超时')
        return future.result()

    def states_of(self, task_id):
        with LOCK:
            return [m.state for m in STATUSES.get(task_id, [])]

    def run_case(self, task_id, wait_state, extra_wait, expect_detail):
        with LOCK:
            ROBOTS.clear()
        self.wait_until(lambda: 'r' in ROBOTS and ROBOTS['r'].state == RobotStatus.STATE_IDLE,
                        15.0, 'agent 空闲')
        with LOCK:
            before_canceled = ROBOTS['r'].tasks_canceled
            before_failed = ROBOTS['r'].tasks_failed

        goal = ExecuteTask.Goal()
        goal.task_id = task_id
        goal.target_color = 1
        goal.target_pose.position.x = 10.0
        goal.target_pose.position.y = 0.0
        goal.target_pose.orientation.w = 1.0
        goal.standoff = 0.6

        goal_handle = self.wait_future(self.client.send_goal_async(goal), 10.0, 'goal 接受')
        self.assertTrue(goal_handle.accepted, f'{task_id} 被拒绝')

        self.wait_until(lambda: wait_state in self.states_of(task_id), 15.0,
                        f'{task_id} 进入状态 {wait_state}')
        time.sleep(extra_wait)

        self.wait_future(goal_handle.cancel_goal_async(), 10.0, 'cancel 请求')
        result = self.wait_future(goal_handle.get_result_async(), 15.0, 'action result')
        self.assertEqual(result.status, GoalStatus.STATUS_CANCELED,
                         f'{task_id} 的 action 终态应为 CANCELED')
        self.assertEqual(result.result.failure_reason, 'CANCELED_BY_OPERATOR')

        self.wait_until(lambda: 6 in self.states_of(task_id), 10.0, 'TaskStatus CANCELED')
        self.assertNotIn(5, self.states_of(task_id), '取消不能上报 FAILED')

        self.wait_until(lambda: ROBOTS['r'].tasks_canceled == before_canceled + 1, 5.0,
                        '取消计数 +1')
        with LOCK:
            details = [m.detail for m in STATUSES.get(task_id, []) if m.state == 6]
            self.assertTrue(any(expect_detail in d for d in details),
                            f'取消 detail 应包含“{expect_detail}”，实际 {details}')
            self.assertEqual(ROBOTS['r'].tasks_failed, before_failed,
                             '取消不能计入 tasks_failed')

    # ---------------- 用例 ----------------

    def test_cancel_in_navigation(self):
        self.run_case('C_nav', STATE_NAVIGATING, 0.0, '导航中')

    def test_cancel_in_docking(self):
        self.run_case('C_dock', STATE_DOCKING, 0.0, '对位中')

    def test_cancel_in_search(self):
        # 无检测输入，对位 1.5s 后进入倒车+转圈搜索
        self.run_case('C_search', STATE_DOCKING, 3.0, '搜索中')


if __name__ == '__main__':
    unittest.main()