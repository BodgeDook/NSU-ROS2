#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <turtlesim/srv/spawn.hpp>

class TurtleTimeFollower : public rclcpp::Node
{
public:
    TurtleTimeFollower() : Node("turtle_time_follower"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->declare_parameter<double>("delay", 5.0);
        delay_ = this->get_parameter("delay").as_double();
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TurtleTimeFollower::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Time follower started with delay: %.1f seconds", delay_);
    }

private:
    void control_loop()
    {
        try {
            auto now = this->get_clock()->now();
            auto past_time = now - rclcpp::Duration::from_seconds(delay_);
            
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_.lookupTransform(
                    "world", "turtle1", past_time, rclcpp::Duration::from_seconds(0.1));
            } catch (tf2::ExtrapolationException &ex) {
                transform = tf_buffer_.lookupTransform("world", "turtle1", tf2::TimePointZero);
            }
            
            auto transform_turtle2 = tf_buffer_.lookupTransform("world", "turtle2", tf2::TimePointZero);
            
            double target_x = transform.transform.translation.x;
            double target_y = transform.transform.translation.y;
            
            double current_x = transform_turtle2.transform.translation.x;
            double current_y = transform_turtle2.transform.translation.y;
            
            double dx = target_x - current_x;
            double dy = target_y - current_y;
            
            double distance = sqrt(dx*dx + dy*dy);
            double angle_to_target = atan2(dy, dx);
            
            double current_theta = 2.0 * atan2(
                transform_turtle2.transform.rotation.z,
                transform_turtle2.transform.rotation.w
            );
            
            double angle_diff = angle_to_target - current_theta;
            
            while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
            
            geometry_msgs::msg::Twist cmd;
            
            if (distance > 0.1) {
                cmd.linear.x = std::min(distance * 0.5, 2.0);
                cmd.angular.z = angle_diff * 4.0;
            } else {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            }
            
            cmd.angular.z = std::max(std::min(cmd.angular.z, 2.0), -2.0);
            
            cmd_pub_->publish(cmd);
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
        }
    }

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double delay_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleTimeFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
