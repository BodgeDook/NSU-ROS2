import math
from geometry_msgs.msg import TransformStamped, Twist
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TurtleTf2Listener(Node):

    def __init__(self):
        super().__init__('turtle_tf2_listener')
        
        # TF2 listener:
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for turtle2 velocity:
        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 1)
        
        # Calling on_timer function every 100ms:
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        # Looking up for the transformation between carrot and turtle2:
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'turtle2',
                'carrot',
                now)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform turtle2 to carrot: {ex}')
            return

        msg = Twist()
        
        # Calculating linear and angular velocity (proportional controller for linear velocity):
        scale_rotation_rate = 1.0
        scale_forward_speed = 0.5
        
        msg.angular.z = scale_rotation_rate * math.atan2(
            trans.transform.translation.y,
            trans.transform.translation.x)
            
        msg.linear.x = scale_forward_speed * math.sqrt(
            trans.transform.translation.x ** 2 +
            trans.transform.translation.y ** 2)
            
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TurtleTf2Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    rclpy.shutdown()
