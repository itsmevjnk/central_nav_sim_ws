import rclpy
from rclpy.node import Node
from rclpy import qos

from std_msgs.msg import String
from gazebo_msgs.msg import ContactsState, ContactState

class BumperNode(Node):
    def __init__(self):
        super().__init__('bumper_node')
        
        self.robot_name = self.declare_parameter('robot_name', 'robot').get_parameter_value().string_value
        
        self.active = None # undetermined
        self.telemetry_pub = self.create_publisher(String, 'telemetry', qos.qos_profile_system_default)

        self.collided_robot = set(); self.collided_static = set()
        self.create_subscription(ContactsState, '/bumper_states', self.bumper_cb, qos.qos_profile_sensor_data)

    def bumper_cb(self, data: ContactsState):
        collided_robot = set(); collided_static = set()
        checked_contacts = 0
        for contact in data.states:
            contact: ContactState
            
            robot_collision = contact.collision1_name.split('::')
            obs_collision = contact.collision2_name.split('::')
            if robot_collision[0] != self.robot_name:
                robot_collision, obs_collision = obs_collision, robot_collision
                if robot_collision[0] != self.robot_name:
                    self.get_logger().warn(f'robot not found in contact state, ignoring')
                    continue
            
            checked_contacts += 1
            obstacle = obs_collision[0]
            if obstacle.startswith('robot'): # robot versus robot collision
                collided_robot.add(obstacle)
            elif obstacle != 'ground_plane': # we don't count ground plane collisions
                collided_static.add(obstacle)
        
        if checked_contacts == 0: return # no contacts to look for

        if self.collided_robot != collided_robot:
            self.collided_robot = collided_robot
            self.telemetry_pub.publish(String(data=f'{self.get_clock().now().nanoseconds}:bumper_telemetry:{self.robot_name},robot,{len(collided_robot) > 0},{",".join(collided_robot)}'))
            self.get_logger().info(f'{self.robot_name} is in collision with {len(collided_robot)} robot(s)')

        if self.collided_static != collided_static:
            self.collided_static = collided_static
            self.telemetry_pub.publish(String(data=f'{self.get_clock().now().nanoseconds}:bumper_telemetry:{self.robot_name},static,{len(collided_static) > 0},{",".join(collided_static)}'))
            self.get_logger().info(f'{self.robot_name} is in collision with {len(collided_static)} static obstacle(s)')
            
def main():
    rclpy.init()
    node = BumperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
