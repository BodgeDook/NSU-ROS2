import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class StarPatternController(Node):
    """
    Publishes Twist messages for the robot to move along the exact Five-pointed Star.
    Uses a 10-phase finite state machine: 5 beams and 5 turns.
    """
    def __init__(self):
        super().__init__('star_pattern_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_period = 0.1 # Control frequency 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # --- MOVEMENT AND PHASE CONSTANTS ---
        self.V_LINEAR = 0.3      # Linear velocity along the beam (m/s)
        self.OMEGA_PIVOT = 0.5   # Angular velocity for turning (rad/s)
        
        # Steps for moving (6.0 s):
        self.MOVE_STEPS = 60 
        # Steps for turning 144 degrees (5.0 s):
        self.PIVOT_STEPS = 50 
        
        # Finite state machine parameters:
        self.current_step = 0
        self.phase = 1 # from 1 to 10. Odd - moving, Even - turning.
        
        self.get_logger().info('StarPatternController Node started. The robot will begin drawing an ENLARGED Five-pointed Star.')

    def timer_callback(self):
        msg = Twist()
        
        # --- MOVEMENT LOGIC (10-phase finite state machine) ---
        
        if self.phase % 2 != 0:
            # Odd phases (1, 3, 5, 7, 9): Moving along the beam:
            msg.linear.x = self.V_LINEAR
            msg.angular.z = 0.0
            max_steps = self.MOVE_STEPS
            
        else:
            # Even phases (2, 4, 6, 8, 10): Turning 144 degrees:
            msg.linear.x = 0.0
            # Set positive angular velocity for turning:
            msg.angular.z = self.OMEGA_PIVOT 
            max_steps = self.PIVOT_STEPS
            
        self.publisher_.publish(msg)
        
        # --- PHASE SWITCHING LOGIC ---
        self.current_step += 1
        
        if self.current_step >= max_steps:
            self.current_step = 0
            self.phase += 1
            
            if self.phase > 10:
                self.phase = 1 # restart the cycle
                self.get_logger().info('Full Star completed. Starting a new cycle.')
            
            # Logging the switch:
            if self.phase % 2 != 0:
                self.get_logger().info(f'Switching: Phase {self.phase}/10 - Moving along the beam ({self.MOVE_STEPS * self.timer_period} s)')
            else:
                self.get_logger().info(f'Switching: Phase {self.phase}/10 - Turning 144° ({self.PIVOT_STEPS * self.timer_period} s)')

def main(args=None):
    rclpy.init(args=args)
    controller = StarPatternController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        controller.publisher_.publish(stop_msg)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()