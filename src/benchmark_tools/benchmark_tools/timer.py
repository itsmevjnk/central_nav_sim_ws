import rclpy
from rclpy.node import Node

class Timer(Node):
    def __init__(self):
        super().__init__('timer')

        self.duration = self.declare_parameter('duration', 0.0).get_parameter_value().double_value
        self.get_logger().info(f'waiting for {self.duration} sec')
        self.create_timer(self.duration, self.timer_cb)

    def timer_cb(self):
        self.get_logger().info(f'time reached, exiting')
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
