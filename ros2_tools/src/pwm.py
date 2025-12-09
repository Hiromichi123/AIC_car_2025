#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

_duty_cycle_map = {
    -1: 12.0,  # 左转
    0: 7.6,    # 中位
    1: 2.8,    # 右转
}

class SteeringPwmNode(Node):
    def __init__(self) -> None:
        super().__init__("steering_pwm_node")

        self._pwm_pin = 13
        self._frequency = 50.0

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pwm_pin, GPIO.OUT)

        self._pwm = GPIO.PWM(self._pwm_pin, self._frequency)
        self._current_duty_cycle = None
        self._set_duty_cycle(_duty_cycle_map[0]) # 初始化为中位

        self._subscription = self.create_subscription(
            Int32,
            '/camera_move',
            self._on_command,
            3,
        )

    def _on_command(self, msg: Int32) -> None:
        duty_cycle = _duty_cycle_map.get(msg.data)

        if duty_cycle is None:
            self.get_logger().warn(
                f"Unknown steering command {msg.data}; expected one of {list(_duty_cycle_map)}"
            )
            return
        self._set_duty_cycle(duty_cycle)

    def _set_duty_cycle(self, duty_cycle: float) -> None:
        if self._current_duty_cycle is None:
            self._pwm.start(duty_cycle)
        else:
            self._pwm.ChangeDutyCycle(duty_cycle)
        self._current_duty_cycle = duty_cycle

    def destroy_node(self) -> bool:
        if self._pwm is not None:
            self._pwm.stop()
            self._pwm = None
        GPIO.cleanup(self._pwm_pin)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SteeringPwmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("关闭旋转舵机")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
