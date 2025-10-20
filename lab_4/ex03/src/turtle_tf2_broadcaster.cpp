#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlesim/msg/pose.hpp>

class TurtleTf2Broadcaster : public rclcpp::Node
{
public:
    TurtleTf2Broadcaster() : Node("turtle_tf2_broadcaster")
    {
        this->declare_parameter<std::string>("turtle_name", "turtle1");
        turtle_name_ = this->get_parameter("turtle_name").as_string();
        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/" + turtle_name_ + "/pose", 10,
            std::bind(&TurtleTf2Broadcaster::handle_turtle_pose, this, std::placeholders::_1));
    }

private:
    void handle_turtle_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
    {
        geometry_msgs::msg::TransformStamped t;
        
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = turtle_name_.c_str();
        
        t.transform.translation.x = msg->x;
        t.transform.translation.y = msg->y;
        t.transform.translation.z = 0.0;
        
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = sin(msg->theta / 2);
        t.transform.rotation.w = cos(msg->theta / 2);
        
        tf_broadcaster_->sendTransform(t);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
    std::string turtle_name_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleTf2Broadcaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
