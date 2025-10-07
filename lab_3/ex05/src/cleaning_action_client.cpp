#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <action_cleaning_robot/action/cleaning_task.hpp>

using CleaningTask = action_cleaning_robot::action::CleaningTask;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<CleaningTask>;

class CleaningActionClient : public rclcpp::Node {
public:
  CleaningActionClient() : Node("cleaning_action_client"), goal_done_(false) {
    this->client_ = rclcpp_action::create_client<CleaningTask>(this, "cleaning_task");

    if (!this->client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        timer_->cancel();
        send_goals();
      });
  }

  bool is_goal_done() const {
    return this->goal_done_;
  }

private:
  rclcpp_action::Client<CleaningTask>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
  int goals_completed_ = 0;

  void send_goals() {
    auto goal_msg1 = CleaningTask::Goal();
    goal_msg1.task_type = "clean_square";
    goal_msg1.area_size = 1.5;
    goal_msg1.target_x = 0.0;
    goal_msg1.target_y = 0.0;

    auto send_goal_options1 = rclcpp_action::Client<CleaningTask>::SendGoalOptions();
    send_goal_options1.goal_response_callback = 
      std::bind(&CleaningActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options1.feedback_callback = 
      std::bind(&CleaningActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options1.result_callback = 
      std::bind(&CleaningActionClient::result_callback, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "Sending goal 1: clean_square 1.5x1.5 meters");
    client_->async_send_goal(goal_msg1, send_goal_options1);
  }

  void goal_response_callback(const ClientGoalHandle::SharedPtr & goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(ClientGoalHandle::SharedPtr,
    const std::shared_ptr<const CleaningTask::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), 
      "Feedback: progress %d%%, cleaned points %d, position (%.2f, %.2f)",
      feedback->progress_percent, 
      feedback->current_cleaned_points, 
      feedback->current_x, 
      feedback->current_y);
  }

  void result_callback(const ClientGoalHandle::WrappedResult & result) {
    goals_completed_++;
    
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), 
          "Goal %d succeeded: success=%d, cleaned_points=%d, total_distance=%.2f",
          goals_completed_,
          result.result->success, 
          result.result->cleaned_points, 
          result.result->total_distance);
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        rclcpp::shutdown();
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        rclcpp::shutdown();
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        rclcpp::shutdown();
        return;
    }

    if (goals_completed_ == 1) {
      auto goal_msg2 = CleaningTask::Goal();
      goal_msg2.task_type = "return_home";
      goal_msg2.area_size = 0.0;
      goal_msg2.target_x = 5.5;
      goal_msg2.target_y = 5.5;

      auto send_goal_options2 = rclcpp_action::Client<CleaningTask>::SendGoalOptions();
      send_goal_options2.goal_response_callback = 
        std::bind(&CleaningActionClient::goal_response_callback, this, std::placeholders::_1);
      send_goal_options2.feedback_callback = 
        std::bind(&CleaningActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options2.result_callback = 
        std::bind(&CleaningActionClient::result_callback, this, std::placeholders::_1);

      RCLCPP_INFO(this->get_logger(), "Sending goal 2: return_home to (5.5, 5.5)");
      client_->async_send_goal(goal_msg2, send_goal_options2);
    } else if (goals_completed_ == 2) {
      goal_done_ = true;
      RCLCPP_INFO(this->get_logger(), "All goals completed successfully!");
      rclcpp::shutdown();
    }
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<CleaningActionClient>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(action_client);
  
  while (!action_client->is_goal_done() && rclcpp::ok()) {
    executor.spin_some(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}
