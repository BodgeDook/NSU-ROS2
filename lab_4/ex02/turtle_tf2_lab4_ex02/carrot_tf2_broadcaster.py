import math
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

class CarrotTf2Broadcaster(Node):

    def __init__(self):
        super().__init__('carrot_tf2_broadcaster')
        
        # Declaring and getting parameters:
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('direction_of_rotation', 1)
        
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.direction = self.get_parameter('direction_of_rotation').get_parameter_value().integer_value
        
        # Validating direction parameter:
        if self.direction not in [1, -1]:
            self.get_logger().warn('Invalid direction_of_rotation, using default value 1')
            self.direction = 1
            
        self.get_logger().info(f'Carrot broadcaster started with radius: {self.radius}, direction: {self.direction}')
        
        # Initializing the transform broadcaster:
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Calling on_timer function every 100ms:
        self.timer = self.create_timer(0.1, self.broadcast_carrot_frame)
        
        self.start_time = self.get_clock().now()

    def broadcast_carrot_frame(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9 # converting to seconds
        
        # Calculating carrot position (rotating around turtle1):
        angle = elapsed_time * self.direction # rotating 1 radian per second
        x = self.radius * math.cos(angle)
        y = self.radius * math.sin(angle)
        
        t = TransformStamped()
        
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot'
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        # Carrot frame has the same orientation as turtle1:
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = CarrotTf2Broadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    rclpy.shutdown()
