import rclpy
from rclpy.node import Node
from rclpy import qos

from action_msgs.msg import GoalStatusArray, GoalStatus

class WaitUntilNavComplete(Node):
    def __init__(self):
        super().__init__('nav2_cancel_stopper')
        
        latched_qos = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            depth=1,
            durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
        ) # same as bt_navigator
        self.create_subscription(GoalStatusArray, 'goal_status', self.status_cb, latched_qos)

        self.navigating: bool = False
        
    def status_cb(self, data: GoalStatusArray):
        if len(data.status_list) == 0: return # nothing to do

        latest_status: GoalStatus = data.status_list[-1] # newest goal is last
        self.get_logger().info(f'latest goal status: {latest_status.status}')

        if not self.navigating and (latest_status.status == 1 or latest_status.status == 2):
            self.get_logger().info(f'navigation started')
            self.navigating = True
            return

        if self.navigating and latest_status.status == 4: # completed
            self.get_logger().info(f'navigation completed, exiting')
            raise SystemExit
            
def main():
    rclpy.init()
    node = WaitUntilNavComplete()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass

    rclpy.shutdown()
