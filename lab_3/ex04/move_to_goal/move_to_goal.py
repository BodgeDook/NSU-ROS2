import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class MoveToGoal(Node):
    def __init__(self, goal_x, goal_y, goal_theta):
        super().__init__('move_to_goal')
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = goal_theta

        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.pose = Pose()
        self.timer = self.create_timer(0.1, self.move_to_goal)
        self.get_logger().info(f'Goal set to: x={goal_x}, y={goal_y}, theta={goal_theta}')

    def pose_callback(self, msg):
        self.pose = msg

    def move_to_goal(self):
        vel_msg = Twist()

        distance = math.sqrt((self.goal_x - self.pose.x)**2 + (self.goal_y - self.pose.y)**2)
        angle_to_goal = math.atan2(self.goal_y - self.pose.y, self.goal_x - self.pose.x)
        angle_error = angle_to_goal - self.pose.theta

        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        k_linear = 1.0
        k_angular = 4.0

        if distance > 0.1:
            vel_msg.linear.x = k_linear * distance
            vel_msg.angular.z = k_angular * angle_error
        else:
            angle_diff = self.goal_theta - self.pose.theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
            if abs(angle_diff) > 0.05:
                vel_msg.angular.z = 2.0 * angle_diff
            else:
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.get_logger().info('Goal reached!')
                self.publisher_.publish(vel_msg)
                rclpy.shutdown()
                return

        self.publisher_.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) != 4:
        print("Usage: ros2 run move_to_goal move_to_goal x y theta")
        return

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    goal_theta = float(sys.argv[3])

    node = MoveToGoal(goal_x, goal_y, goal_theta)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
