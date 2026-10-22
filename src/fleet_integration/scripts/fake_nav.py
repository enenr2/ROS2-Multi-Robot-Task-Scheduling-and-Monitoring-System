#!/usr/bin/env python3
"""集成测试用的假 Nav2 导航服务：延迟一段时间后成功，期间接受取消。

真 Nav2 无法进 CI。这个替身只实现 `NavigateToPose` 契约，用来让真
`fleet_agent` 走到导航/对位/搜索三个阶段，从而验证取消语义。
"""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose


class FakeNav(Node):
    def __init__(self):
        super().__init__('fake_nav')
        self.declare_parameter('delay_s', 2.0)
        self.delay_s = self.get_parameter('delay_s').value
        self.group = ReentrantCallbackGroup()
        self.server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=self.group)
        self.get_logger().info(f'假 Nav2 就绪: delay={self.delay_s}s')

    def execute(self, goal_handle):
        start = time.time()
        while time.time() - start < self.delay_s:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('导航目标被取消')
                return NavigateToPose.Result()
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = max(0.0, 1.0 - (time.time() - start) / self.delay_s)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.1)
        goal_handle.succeed()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = FakeNav()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()