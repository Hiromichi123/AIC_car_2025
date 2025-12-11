#!/usr/bin/env python3
import Jetson.GPIO as GPIO
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from ros2_tools.srv import SERVO

_duty_cycle_map = {
    -1: 12.0,  # 左转
    0: 7.6,    # 中位
    1: 2.8,    # 右转
}

class SteeringPwmNode(Node):
    def __init__(self) -> None:
        super().__init__("steering_pwm_node")

        self.declare_parameter("service", "servo_control")

        self._pwm_pin = 13
        self._frequency = 50.0
        service_name = self.get_parameter("service").get_parameter_value().string_value

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self._pwm_pin, GPIO.OUT)

        self._pwm = GPIO.PWM(self._pwm_pin, self._frequency)
        self._current_duty_cycle = None
        self._set_duty_cycle(_duty_cycle_map[0])  # 初始化为中位

        self._cb_group = ReentrantCallbackGroup()
        self._service = self.create_service(
            SERVO, service_name, self._on_command, callback_group=self._cb_group
        )

        self.get_logger().info(
            f"servo service '{service_name}' ready (pin {self._pwm_pin}, {self._frequency}Hz)"
        )

    def _on_command(self, request: SERVO.Request, response: SERVO.Response):
        duty_cycle = _duty_cycle_map.get(request.command)

        if duty_cycle is None:
            response.success = False
            response.message = (
                f"Unknown steering command {request.command}; expected one of {list(_duty_cycle_map)}"
            )
            self.get_logger().warn(response.message)
            return response

        self._set_duty_cycle(duty_cycle)
        self.get_logger().info(
            f"Applied command {request.command} (duty {duty_cycle:.2f}%), waiting 1s before response"
        )
        self.create_timer(1.0, lambda: None)  # ensure callbacks run before sleep
        self._sleep_non_blocking(1.0)
        response.success = True
        response.message = f"Applied command {request.command} (duty {duty_cycle:.2f}%)"
        self.get_logger().info(response.message)
        return response

    def _sleep_non_blocking(self, seconds: float) -> None:
        end_time = self.get_clock().now() + rclpy.time.Duration(seconds=seconds)
        while self.get_clock().now() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

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
