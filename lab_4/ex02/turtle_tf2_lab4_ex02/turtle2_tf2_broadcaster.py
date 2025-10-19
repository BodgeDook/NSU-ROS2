import math
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose

class Turtle2Tf2Broadcaster(Node):

    def __init__(self):
        super().__init__('turtle2_tf2_broadcaster')
        
        # Initializing the transform broadcaster:
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribing to turtle2 pose:
        self.subscription = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.handle_turtle_pose,
            1)
        
        self.get_logger().info('Started turtle2_tf2_broadcaster')

    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        
        # Reading message content and assign it to corresponding tf variables:
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle2'
        
        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0:
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        
        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in quaternion:
        q = self.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Sending the transformation:
        self.tf_broadcaster.sendTransform(t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = [0] * 4
        q[0] = sr * cp * cy - cr * sp * sy
        q[1] = cr * sp * cy + sr * cp * sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        
        return q

def main():
    rclpy.init()
    node = Turtle2Tf2Broadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    rclpy.shutdown()
