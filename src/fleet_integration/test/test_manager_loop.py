"""调度中心 ↔ 执行代理的集成测试（不依赖 Gazebo / Nav2）。

被测对象是真实的 `fleet_manager_node`，执行代理换成 `fake_agent`：
它按 `ExecuteTask` 契约发布心跳、反馈、终态，并能配合取消。这样 CI 里
也能验证"检测 → 建单 → 派单 → action → 终态"和"取消"两条主链路。

跑法：
    colcon test --packages-select fleet_integration --event-handlers console_direct+
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

from fleet_msgs.msg import Detection, RobotStatus, Task, TaskStatus
from fleet_msgs.srv import CancelTask, SubmitTask

TASKS = {}
STATUSES = []
ROBOT_STATUS = {}
# 可重入锁：wait_until 持锁判断时，谓词内部还会调用 snapshot()/tasks_with_color()
LOCK = threading.RLock()


def _on_task(msg):
    with LOCK:
        TASKS[msg.task_id] = msg


def _on_task_status(msg):
    with LOCK:
        STATUSES.append(msg)


def _on_robot_status(msg):
    with LOCK:
        ROBOT_STATUS[msg.robot_id] = msg


def generate_test_description():
    fake_agent = launch_ros.actions.Node(
        package='fleet_integration',
        executable='fake_agent',
        namespace='robot1',
        parameters=[{'robot_id': 'robot1', 'hold_far_targets': True}],
        output='screen')

    manager = launch_ros.actions.Node(
        package='fleet_manager',
        executable='fleet_manager_node',
        name='fleet_manager',
        parameters=[{
            'robot_ids': ['robot1'],
            'dispatch_robot_ids': ['robot1'],
            'standoff': 0.6,
            'publish_markers': False,
        }],
        output='screen')

    return launch.LaunchDescription([
        fake_agent,
        manager,
        launch_testing.actions.ReadyToTest(),
    ]), {}


class ManagerAgentLoopTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('integration_probe')
        cls.node.create_subscription(Task, '/fleet/tasks', _on_task, 20)
        cls.node.create_subscription(TaskStatus, '/robot1/task_status', _on_task_status, 50)
        cls.node.create_subscription(RobotStatus, '/robot1/robot_status', _on_robot_status, 20)
        cls.detection_pub = cls.node.create_publisher(Detection, '/robot1/detections', 10)
        cls.submit_client = cls.node.create_client(SubmitTask, '/fleet/submit_task')
        cls.cancel_client = cls.node.create_client(CancelTask, '/fleet/cancel_task')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    # ---------------- 工具 ----------------

    def wait_until(self, predicate, timeout, description):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with LOCK:
                if predicate():
                    return
            time.sleep(0.05)
        self.fail(f'等待超时（{timeout:.0f}s）：{description}')

    def wait_for_service(self, client, name):
        self.assertTrue(client.wait_for_service(timeout_sec=15.0), f'{name} 不可用')

    def call(self, client, request, timeout=15.0):
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _future: done.set())
        self.assertTrue(done.wait(timeout), '服务调用超时')
        return future.result()

    def snapshot(self, task_id):
        with LOCK:
            return TASKS.get(task_id)

    def publish_detection(self, color, x, y, lost=False):
        msg = Detection()
        msg.header.frame_id = 'map'
        msg.source_robot_id = 'robot1'
        msg.color = color
        msg.confidence = 1.0
        msg.lost = lost
        msg.world_pose.x = x
        msg.world_pose.y = y
        msg.world_pose.z = 0.1
        msg.range = -1.0 if lost else 3.2
        msg.bearing = 0.0
        msg.range_sigma = 0.1
        for _ in range(3):
            self.detection_pub.publish(msg)
            time.sleep(0.15)

    def tasks_with_color(self, color):
        with LOCK:
            return [task for task in TASKS.values() if task.target_color == color]

    # ---------------- 用例 ----------------

    def test_1_detection_creates_and_completes_task(self):
        """检测到目标 → 建单 → 派给 robot1 → DONE。"""
        self.wait_until(
            lambda: ROBOT_STATUS.get('robot1') is not None
            and ROBOT_STATUS['robot1'].heartbeat_seq >= 2,
            20.0, '假代理心跳上线')

        self.publish_detection(Detection.COLOR_RED, 3.0, 1.0)
        self.wait_until(
            lambda: any(task.assigned_robot_id == 'robot1' for task in self.tasks_with_color(
                Detection.COLOR_RED)),
            20.0, '红色目标建单并派单')

        red_tasks = self.tasks_with_color(Detection.COLOR_RED)
        self.assertEqual(len(red_tasks), 1, '同色同位置只应建一个任务')
        task_id = red_tasks[0].task_id

        self.wait_until(lambda: self.snapshot(task_id).state == TaskStatus.STATE_DONE,
                        20.0, f'{task_id} 进入 DONE')

        with LOCK:
            states = [msg.state for msg in STATUSES if msg.task_id == task_id]
        self.assertIn(TaskStatus.STATE_NAVIGATING, states)
        self.assertIn(TaskStatus.STATE_DOCKING, states)
        self.assertEqual(states[-1], TaskStatus.STATE_DONE)

    def test_2_lost_detection_is_ignored(self):
        """丢失事件（lost=true / range<0）不建单。"""
        self.publish_detection(Detection.COLOR_GREEN, 5.0, 5.0, lost=True)
        time.sleep(2.0)
        self.assertEqual(self.tasks_with_color(Detection.COLOR_GREEN), [],
                         '丢失事件不应建出任何绿色任务')

    def test_3_manual_task_can_be_canceled(self):
        """服务注入任务 → 派单 → CancelTask → CANCELED，并释放车辆。"""
        self.wait_for_service(self.submit_client, 'SubmitTask')
        self.wait_for_service(self.cancel_client, 'CancelTask')

        request = SubmitTask.Request()
        request.target_color = Detection.COLOR_BLUE
        request.target_pose.position.x = 200.0
        request.target_pose.position.y = 0.0
        request.target_pose.orientation.w = 1.0
        response = self.call(self.submit_client, request)
        self.assertTrue(response.accepted, response.message)
        task_id = response.task_id

        self.wait_until(
            lambda: self.snapshot(task_id) is not None
            and self.snapshot(task_id).assigned_robot_id == 'robot1'
            and self.snapshot(task_id).state in (TaskStatus.STATE_ASSIGNED,
                                                 TaskStatus.STATE_NAVIGATING,
                                                 TaskStatus.STATE_DOCKING),
            20.0, f'{task_id} 进入执行中')

        cancel_request = CancelTask.Request()
        cancel_request.task_id = task_id
        cancel_request.reason = 'CANCELED_BY_OPERATOR'
        cancel_response = self.call(self.cancel_client, cancel_request)
        self.assertTrue(cancel_response.canceled, cancel_response.message)

        self.wait_until(lambda: self.snapshot(task_id).state == TaskStatus.STATE_CANCELED,
                        15.0, f'{task_id} 进入 CANCELED')
        self.assertEqual(self.snapshot(task_id).failure_reason, 'CANCELED_BY_OPERATOR')

        self.wait_until(
            lambda: ROBOT_STATUS['robot1'].state == RobotStatus.STATE_IDLE
            and not ROBOT_STATUS['robot1'].current_task,
            10.0, '车辆回到空闲')

        # 取消单独计数，不能混进 tasks_failed
        self.wait_until(lambda: ROBOT_STATUS['robot1'].tasks_canceled >= 1, 10.0,
                        '取消计数 +1')
        self.assertEqual(ROBOT_STATUS['robot1'].tasks_failed, 0, '取消不应计入失败')

    def test_4_duplicate_submit_is_rejected(self):
        """与执行中任务同色同位置（<0.8m）的注入应被拒绝。"""
        self.wait_for_service(self.submit_client, 'SubmitTask')
        request = SubmitTask.Request()
        request.target_color = Detection.COLOR_GREEN
        request.target_pose.position.x = 300.0
        request.target_pose.position.y = 0.0
        request.target_pose.orientation.w = 1.0
        first = self.call(self.submit_client, request)
        self.assertTrue(first.accepted, first.message)

        duplicate = self.call(self.submit_client, request)
        self.assertFalse(duplicate.accepted, '执行中的重复注入不应被接受')

        # 收尾：取消掉这个占位任务，避免一直占着车
        cancel_request = CancelTask.Request()
        cancel_request.task_id = first.task_id
        cancel_request.reason = 'CANCELED_BY_OPERATOR'
        self.assertTrue(self.call(self.cancel_client, cancel_request).canceled)


if __name__ == '__main__':
    unittest.main()