import rclpy
from rclpy.node import Node

class Timer(Node):
    def __init__(self):
        super().__init__('timer')

        self.duration = self.declare_parameter('duration', 0.0).get_parameter_value().double_value
        self.kickstart_timer = self.create_timer(0.1, self.start_timer_cb)

    def start_timer_cb(self):
        self.get_logger().info(f'waiting for {self.duration} sec - starting at {self.get_clock().now().nanoseconds}')
        self.create_timer(self.duration, self.timer_cb)
        self.kickstart_timer.cancel()

    def timer_cb(self):
        self.get_logger().info(f'time reached (at {self.get_clock().now().nanoseconds}), exiting')
        raise SystemExit

def main():
    rclpy.init()
    node = Timer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass

    rclpy.shutdown()
