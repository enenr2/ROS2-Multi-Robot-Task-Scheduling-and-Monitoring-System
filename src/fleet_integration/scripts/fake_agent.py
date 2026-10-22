#!/usr/bin/env python3
"""集成测试用的假执行代理：只实现接口，不跑导航与视觉。

真 `fleet_agent` 依赖 Nav2、Gazebo 和相机，无法进 CI。这里用同一个
`ExecuteTask` action 契约做一个可控替身，用来验证调度层的行为：

  * 发布 `/<ns>/robot_status`（5 Hz），让调度器认为车在线
  * 提供 `/<ns>/execute_task`：普通目标 1s 内走完
    ASSIGNED → NAVIGATING → DOCKING → DONE 并回写 `task_status`
  * 目标 x > 100 的任务一直保持执行中，直到收到取消请求 —— 用于验证
    `CancelTask` 服务 → action cancel 的整条链路

它不模拟故障；故障注入仍由 scripts/fault_injector.py 在真仿真上做。
"""
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fleet_msgs.action import ExecuteTask
from fleet_msgs.msg import RobotStatus, TaskStatus

# 与 TaskStatus.msg 的 STATE_* 保持一致
STATE_ASSIGNED = 1
STATE_NAVIGATING = 2
STATE_DOCKING = 3
STATE_DONE = 4
STATE_CANCELED = 6


class FakeAgent(Node):
    def __init__(self):
        super().__init__('fake_agent')
        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').value

        self.busy = False
        self.current_task = ''
        self.tasks_done = 0
        self.seq = 0
        self.mutex = threading.Lock()
        self.group = ReentrantCallbackGroup()

        self.status_pub = self.create_publisher(RobotStatus, 'robot_status', 10)
        self.task_status_pub = self.create_publisher(TaskStatus, 'task_status', 20)
        self.create_timer(0.2, self.publish_status, callback_group=self.group)

        self.server = ActionServer(
            self, ExecuteTask, 'execute_task',
            execute_callback=self.execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=self.group)
        self.get_logger().info(f'假执行代理就绪: {self.robot_id}')

    # ---------------- 心跳 ----------------

    def publish_status(self):
        msg = RobotStatus()
        msg.robot_id = self.robot_id
        msg.state = RobotStatus.STATE_BUSY if self.busy else RobotStatus.STATE_IDLE
        msg.pose.position.x = 1.0
        msg.pose.position.y = 0.0
        msg.pose.orientation.w = 1.0
        with self.mutex:
            msg.current_task = self.current_task
            msg.tasks_done = self.tasks_done
            msg.heartbeat_seq = self.seq
            self.seq += 1
        msg.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(msg)

    def publish_task_status(self, task_id, state, progress, detail, reason='NONE'):
        msg = TaskStatus()
        msg.task_id = task_id
        msg.robot_id = self.robot_id
        msg.state = state
        msg.progress = float(progress)
        msg.detail = detail
        msg.failure_reason = reason
        msg.retry_count = 0
        msg.stamp = self.get_clock().now().to_msg()
        self.task_status_pub.publish(msg)

    # ---------------- action ----------------

    def execute(self, goal_handle):
        goal = goal_handle.request
        with self.mutex:
            self.busy = True
            self.current_task = goal.task_id
        self.publish_status()

        result = ExecuteTask.Result()
        try:
            if goal.target_pose.position.x > 100.0:
                # 停在执行中，等调度层取消
                self.publish_task_status(goal.task_id, STATE_NAVIGATING, 0.2, '等待取消')
                deadline = time.time() + 20.0
                while time.time() < deadline and not goal_handle.is_cancel_requested:
                    time.sleep(0.05)
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.failure_reason = 'CANCELED_BY_OPERATOR'
                    self.publish_task_status(goal.task_id, STATE_CANCELED, 0.0,
                                             '任务被取消', 'CANCELED_BY_OPERATOR')
                else:
                    goal_handle.abort()
                    result.success = False
                    result.failure_reason = 'NAV_TIMEOUT'
                    self.publish_task_status(goal.task_id, TaskStatus.STATE_FAILED, 0.0,
                                             '测试替身超时', 'NAV_TIMEOUT')
                return result

            for state, progress, detail in (
                    (STATE_ASSIGNED, 0.0, '任务已接受'),
                    (STATE_NAVIGATING, 0.3, '导航中'),
                    (STATE_DOCKING, 0.8, '对位中')):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.failure_reason = 'CANCELED_BY_OPERATOR'
                    self.publish_task_status(goal.task_id, STATE_CANCELED, 0.0,
                                             '任务被取消', 'CANCELED_BY_OPERATOR')
                    return result
                feedback = ExecuteTask.Feedback()
                feedback.state = state
                feedback.progress = float(progress)
                feedback.distance_remaining = 1.0
                feedback.detail = detail
                goal_handle.publish_feedback(feedback)
                self.publish_task_status(goal.task_id, state, progress, detail)
                time.sleep(0.3)

            goal_handle.succeed()
            result.success = True
            result.failure_reason = 'NONE'
            result.final_distance = 0.6
            result.final_heading_error = 0.0
            self.publish_task_status(goal.task_id, STATE_DONE, 1.0, '任务完成')
            with self.mutex:
                self.tasks_done += 1
            return result
        finally:
            with self.mutex:
                self.busy = False
                self.current_task = ''
            self.publish_status()


def main():
    rclpy.init()
    node = FakeAgent()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass   # launch_testing 收尾时发 SIGINT，属于正常退出
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()