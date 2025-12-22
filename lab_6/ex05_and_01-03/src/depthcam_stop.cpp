#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>

class DepthCamStop : public rclcpp::Node {
public:
    DepthCamStop() : Node("depthcam_stop") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        rclcpp::QoS qos(rclcpp::KeepLast(10));
        qos.best_effort();
        
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/depth/image_raw", qos,
            std::bind(&DepthCamStop::depth_callback, this, std::placeholders::_1)
        );
        
        start_time_ = std::chrono::high_resolution_clock::now();
        last_log_time_ = start_time_;
        RCLCPP_INFO(this->get_logger(), "Waiting 3 seconds (Depth Cam Mode)...");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point last_log_time_;
    const double LOG_THROTTLE_DURATION = 1.0;
    bool moving = false;

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        auto current_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration<double>(current_time - start_time_).count();
        
        if (elapsed < 3.0) return;

        auto cmd = std::make_unique<geometry_msgs::msg::Twist>();
        
        float center_dist = std::numeric_limits<float>::max();

        if (msg->encoding == "32FC1") {
            const float* depths = reinterpret_cast<const float*>(&msg->data[0]);
            
            int u = msg->width / 2;
            int v = msg->height / 2;
            int center_idx = v * msg->width + u;
            
            float val = depths[center_idx];

            if (!std::isnan(val) && !std::isinf(val) && val > 0.0) {
                center_dist = val;
            }
        } else {
             RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "Unexpected encoding: %s. Expected 32FC1", msg->encoding.c_str());
             return;
        }

        const float STOP_DIST = 1.0f;
        
        if (center_dist < STOP_DIST) {
            cmd->linear.x = 0.0;
            if (moving) {
                // RCLCPP_WARN(this->get_logger(), "Obstacle at %.2f m - STOPPED", center_dist);
                moving = false;
            }
        } else {
            cmd->linear.x = 0.3;
            if (!moving) {
                auto throttle_elapsed = std::chrono::duration<double>(current_time - last_log_time_).count();
                if (throttle_elapsed >= LOG_THROTTLE_DURATION) {
                    RCLCPP_INFO(this->get_logger(), "Moving (clear path: %.2fm)", center_dist);
                    last_log_time_ = current_time;
                    moving = true;
                }
            }
        }

        cmd_vel_pub_->publish(*cmd);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthCamStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
