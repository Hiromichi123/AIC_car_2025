"""简单定时器发布目标点，代替rust版本用作实机测试"""
import math
from dataclasses import dataclass
from typing import List, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


@dataclass
class GoalPose:
    x: float
    y: float
    z: float
    yaw: float


class SimpleGoalPublisher(Node):
    def __init__(self) -> None:
        super().__init__('simple_goal_publisher')

        default_goals = ['0.0,0.0,0.0', '2.0,0.0,0.0', '2.0,1.0,1.57']
        self.declare_parameter('goal_points', default_goals)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_interval', 3.0)
        self.declare_parameter('loop', False)

        goal_strings = self.get_parameter('goal_points').get_parameter_value().string_array_value
        self.goals = self._parse_goals(goal_strings)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_interval = float(self.get_parameter('publish_interval').value)
        self.loop = bool(self.get_parameter('loop').value)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)
        self.current_index = 0
        self.timer = None

        if not self.goals:
            self.get_logger().warning('未提供有效的goal_points')
            return

        self.timer = self.create_timer(self.publish_interval, self._on_timer)
        self.publish_next_goal(initial=True)

    def _parse_goals(self, goal_strings: Sequence[str]) -> List[GoalPose]:
        goals: List[GoalPose] = []
        for raw in goal_strings:
            parts = [p.strip() for p in raw.split(',') if p.strip()]
            if not parts:
                continue
            try:
                numbers = [float(value) for value in parts]
            except ValueError:
                self.get_logger().warning(f'无法解析目标"{raw}"，跳过。')
                continue

            if len(numbers) == 3:
                x, y, yaw = numbers
                goals.append(GoalPose(x=x, y=y, z=0.0, yaw=yaw))
            elif len(numbers) == 4:
                x, y, z, yaw = numbers
                goals.append(GoalPose(x=x, y=y, z=z, yaw=yaw))
            else:
                self.get_logger().warning(
                    f'Goal"{raw}"形式不正确，跳过'
                )

        return goals

    def _on_timer(self) -> None:
        published = self.publish_next_goal()
        if not published and self.timer is not None:
            self.timer.cancel()
            self.timer = None

    def publish_next_goal(self, initial: bool = False) -> bool:
        if not self.goals:
            return False

        if self.current_index >= len(self.goals):
            if self.loop:
                self.current_index = 0
            else:
                if not initial:
                    self.get_logger().info('所有目标已发布，停止发布。')
                return False

        goal = self.goals[self.current_index]
        self.current_index += 1

        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = goal.x
        msg.pose.position.y = goal.y
        msg.pose.position.z = goal.z

        half_yaw = goal.yaw * 0.5
        msg.pose.orientation.w = math.cos(half_yaw)
        msg.pose.orientation.z = math.sin(half_yaw)

        self.goal_pub.publish(msg)
        self.get_logger().info(
            f'Published goal {self.current_index}/{len(self.goals)} '
            f'(x={goal.x:.2f}, y={goal.y:.2f}, z={goal.z:.2f}, yaw={goal.yaw:.2f})'
        )

        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimpleGoalPublisher()
    try:
        if node.timer is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.timer is not None:
            node.timer.cancel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()