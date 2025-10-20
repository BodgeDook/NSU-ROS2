#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

class KeyboardListener : public rclcpp::Node
{
public:
  KeyboardListener()
  : Node("keyboard_listener")
  {
    switch_service_ = this->create_service<std_srvs::srv::Empty>(
      "/switch_target",
      std::bind(&KeyboardListener::switch_target_callback, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Switch service available at /switch_target");
    RCLCPP_INFO(this->get_logger(), "Use: ros2 service call /switch_target std_srvs/srv/Empty");
  }

private:
  void switch_target_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(this->get_logger(), "Manual target switch requested via service");
  }

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr switch_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
