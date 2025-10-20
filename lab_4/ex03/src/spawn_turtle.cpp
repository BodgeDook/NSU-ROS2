#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

class SpawnTurtle : public rclcpp::Node
{
public:
    SpawnTurtle() : Node("spawn_turtle")
    {
        this->declare_parameter<std::string>("turtle_name", "turtle2");
        this->declare_parameter<double>("x", 5.0);
        this->declare_parameter<double>("y", 5.0);
        
        auto turtle_name = this->get_parameter("turtle_name").as_string();
        auto x = this->get_parameter("x").as_double();
        auto y = this->get_parameter("y").as_double();
        
        auto client = this->create_client<turtlesim::srv::Spawn>("/spawn");
        
        while (!client->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for spawn service...");
        }
        
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = turtle_name;
        request->x = x;
        request->y = y;
        request->theta = 0.0;
        
        auto future = client->async_send_request(request);
        
        RCLCPP_INFO(this->get_logger(), "Spawned turtle: %s at (%.1f, %.1f)", 
                   turtle_name.c_str(), x, y);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpawnTurtle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
