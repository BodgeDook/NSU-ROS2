import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMovementPublisher(Node):
    """
    Publishes Twist messages for the robot to move in a circle.
    """
    def __init__(self):
        super().__init__('circle_movement_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1 # 10 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Velocities for moving in a circle:
        self.linear_speed = 0.2  # Linear velocity (m/s)
        self.angular_speed = 0.1 # Angular velocity (r/s)
        
        self.get_logger().info(f'Node CircleMovement was launched. The Robot will start its circle movement.')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    circle_movement_publisher = CircleMovementPublisher()
    rclpy.spin(circle_movement_publisher)
    circle_movement_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()