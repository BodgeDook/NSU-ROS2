import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TextToCmdVelNode(Node):
    def __init__(self):
        super().__init__('text_to_cmd_vel')
        # subscribe on /cmd_text (std_msgs/String)
        self.subscription = self.create_subscription(
            String,
            '/cmd_text',
            self.cmd_text_callback,
            10)
        self.subscription # preventing unused variable warning

        # publication in /turtle1/cmd_vel (geometry_msgs/Twist)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('Text to CmdVel node started. Subscribed to /cmd_text.')

    def cmd_text_callback(self, msg):
        cmd = msg.data.strip().lower()
        twist = Twist()

        if cmd == "move_forward":
            twist.linear.x = 1.0
            twist.angular.z = 0.0
        elif cmd == "move_backward":
            twist.linear.x = -1.0
            twist.angular.z = 0.0
        elif cmd == "turn_left":
            twist.linear.x = 0.0
            twist.angular.z = 1.5 # not the ros standart
        elif cmd == "turn_right":
            twist.linear.x = 0.0
            twist.angular.z = -1.5 # not the ros standart
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().warn(f'Unknown command: "{cmd}". Stopping.')

        self.publisher.publish(twist)
        self.get_logger().info(f'Command "{cmd}" -> Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = TextToCmdVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
