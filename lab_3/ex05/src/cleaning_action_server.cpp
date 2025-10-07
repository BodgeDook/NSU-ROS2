#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <action_cleaning_robot/action/cleaning_task.hpp>
#include <functional>
#include <memory>
#include <thread>
#include <cmath>

using CleaningTask = action_cleaning_robot::action::CleaningTask;
using GoalHandleCleaningTask = rclcpp_action::ServerGoalHandle<CleaningTask>;

class CleaningActionServer : public rclcpp::Node {
public:
  CleaningActionServer() : Node("cleaning_action_server") {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<CleaningTask>(
      this,
      "cleaning_task",
      std::bind(&CleaningActionServer::handle_goal, this, _1, _2),
      std::bind(&CleaningActionServer::handle_cancel, this, _1),
      std::bind(&CleaningActionServer::handle_accepted, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&CleaningActionServer::pose_callback, this, _1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&CleaningActionServer::timer_callback, this));
  }

private:
  rclcpp_action::Server<CleaningTask>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  turtlesim::msg::Pose current_pose_;
  bool pose_updated_ = false;

  std::shared_ptr<GoalHandleCleaningTask> current_goal_handle_;
  double start_x_, start_y_;
  double total_distance_ = 0.0;
  int cleaned_points_ = 0;
  double last_x_ = 0.0, last_y_ = 0.0;
  bool first_pose_ = true;

  // Parameters for tasks
  double area_size_;
  std::string task_type_;

  // For square cleaning
  enum class SquareState { 
    MOVE_RIGHT, 
    MOVE_UP, 
    MOVE_LEFT, 
    MOVE_DOWN,
    COMPLETE
  };
  SquareState square_state_ = SquareState::MOVE_RIGHT;
  int square_passes_ = 0;
  int total_square_passes_;
  double square_step_ = 0.5;

  // For return home
  double home_x_ = 5.5;
  double home_y_ = 5.5;
  double distance_threshold_ = 0.1;

  double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CleaningTask::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with task_type: %s", goal->task_type.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCleaningTask> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    stop_robot();
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCleaningTask> goal_handle) {
    std::thread(&CleaningActionServer::execute, this, goal_handle).detach();
  }

  void execute(const std::shared_ptr<GoalHandleCleaningTask> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");
    current_goal_handle_ = goal_handle;
    const auto goal = goal_handle->get_goal();
    task_type_ = goal->task_type;
    area_size_ = goal->area_size;
    home_x_ = goal->target_x;
    home_y_ = goal->target_y;

    total_distance_ = 0.0;
    cleaned_points_ = 0;
    first_pose_ = true;

    while (!pose_updated_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    start_x_ = current_pose_.x;
    start_y_ = current_pose_.y;
    last_x_ = start_x_;
    last_y_ = start_y_;
    pose_updated_ = false;

    if (task_type_ == "clean_square") {
      initialize_square_cleaning();
    } else if (task_type_ == "return_home") {
      // No init needed
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unknown task type");
      auto result = std::make_shared<CleaningTask::Result>();
      result->success = false;
      goal_handle->abort(result);
      return;
    }
  }

  void initialize_square_cleaning() {
    total_square_passes_ = static_cast<int>(area_size_ / square_step_);
    square_passes_ = 0;
    square_state_ = SquareState::MOVE_RIGHT;
    RCLCPP_INFO(this->get_logger(), "Starting square cleaning: size=%.1f, passes=%d", area_size_, total_square_passes_);
  }

  void timer_callback() {
    if (!current_goal_handle_ || !current_goal_handle_->is_active()) {
      return;
    }

    update_distance_and_points();

    auto feedback = std::make_shared<CleaningTask::Feedback>();
    feedback->current_x = current_pose_.x;
    feedback->current_y = current_pose_.y;
    feedback->current_cleaned_points = cleaned_points_;
    feedback->progress_percent = calculate_progress();

    current_goal_handle_->publish_feedback(feedback);

    if (is_task_complete()) {
      stop_robot();
      auto result = std::make_shared<CleaningTask::Result>();
      result->success = true;
      result->cleaned_points = cleaned_points_;
      result->total_distance = total_distance_;
      current_goal_handle_->succeed(result);
      current_goal_handle_ = nullptr;
      RCLCPP_INFO(this->get_logger(), "Task completed successfully!");
      return;
    }

    move_robot();
  }

  void move_robot() {
    geometry_msgs::msg::Twist twist;

    if (task_type_ == "clean_square") {
      move_square(twist);
    } else if (task_type_ == "return_home") {
      move_to_home(twist);
    }

    publisher_->publish(twist);
  }

  void move_square(geometry_msgs::msg::Twist &twist) {
    const double linear_speed = 1.0;
    const double angular_speed = 1.0;
    const double tolerance = 0.2;

    // Boundary checks
    if (current_pose_.x > 10.5 || current_pose_.x < 0.5 || 
        current_pose_.y > 10.5 || current_pose_.y < 0.5) {
      twist.linear.x = 0.0;
      twist.angular.z = 1.0;
      return;
    }

    switch (square_state_) {
      case SquareState::MOVE_RIGHT: {
        double target_x = start_x_ + area_size_;
        if (current_pose_.x < target_x - tolerance) {
          double angle_diff = normalize_angle(0.0 - current_pose_.theta);
          if (std::abs(angle_diff) > 0.1) {
            twist.linear.x = 0.0;
            twist.angular.z = angular_speed * (angle_diff > 0 ? 1 : -1);
          } else {
            twist.linear.x = linear_speed;
            twist.angular.z = 0.0;
          }
        } else {
          square_state_ = SquareState::MOVE_UP;
          RCLCPP_INFO(this->get_logger(), "Reached right edge, moving up");
        }
        break;
      }

      case SquareState::MOVE_UP: {
        double target_y = start_y_ + area_size_;
        if (current_pose_.y < target_y - tolerance) {
          double angle_diff = normalize_angle(M_PI/2 - current_pose_.theta);
          if (std::abs(angle_diff) > 0.1) {
            twist.linear.x = 0.0;
            twist.angular.z = angular_speed * (angle_diff > 0 ? 1 : -1);
          } else {
            twist.linear.x = linear_speed;
            twist.angular.z = 0.0;
          }
        } else {
          square_state_ = SquareState::MOVE_LEFT;
          RCLCPP_INFO(this->get_logger(), "Reached top edge, moving left");
        }
        break;
      }

      case SquareState::MOVE_LEFT: {
        double target_x = start_x_;
        if (current_pose_.x > target_x + tolerance) {
          double angle_diff = normalize_angle(M_PI - current_pose_.theta);
          if (std::abs(angle_diff) > 0.1) {
            twist.linear.x = 0.0;
            twist.angular.z = angular_speed * (angle_diff > 0 ? 1 : -1);
          } else {
            twist.linear.x = linear_speed;
            twist.angular.z = 0.0;
          }
        } else {
          square_state_ = SquareState::MOVE_DOWN;
          RCLCPP_INFO(this->get_logger(), "Reached left edge, moving down");
        }
        break;
      }

      case SquareState::MOVE_DOWN: {
        double target_y = start_y_;
        if (current_pose_.y > target_y + tolerance) {
          double angle_diff = normalize_angle(-M_PI/2 - current_pose_.theta);
          if (std::abs(angle_diff) > 0.1) {
            twist.linear.x = 0.0;
            twist.angular.z = angular_speed * (angle_diff > 0 ? 1 : -1);
          } else {
            twist.linear.x = linear_speed;
            twist.angular.z = 0.0;
          }
        } else {
          square_passes_++;
          if (square_passes_ >= total_square_passes_) {
            square_state_ = SquareState::COMPLETE;
            RCLCPP_INFO(this->get_logger(), "Square cleaning completed");
          } else {
            start_x_ += square_step_ / 2;
            start_y_ += square_step_ / 2;
            area_size_ -= square_step_;
            square_state_ = SquareState::MOVE_RIGHT;
            RCLCPP_INFO(this->get_logger(), "Starting inner square pass %d", square_passes_ + 1);
          }
        }
        break;
      }

      case SquareState::COMPLETE:
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        break;
    }
  }

  void move_to_home(geometry_msgs::msg::Twist &twist) {
    double dx = home_x_ - current_pose_.x;
    double dy = home_y_ - current_pose_.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    if (distance < distance_threshold_) {
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      return;
    }

    double desired_angle = std::atan2(dy, dx);
    double angle_diff = normalize_angle(desired_angle - current_pose_.theta);

    if (std::abs(angle_diff) > 0.1) {
      twist.linear.x = 0.0;
      twist.angular.z = (angle_diff > 0) ? 0.5 : -0.5;
    } else {
      twist.linear.x = std::min(distance, 1.0);
      twist.angular.z = 0.0;
    }
  }

  bool is_task_complete() {
    if (task_type_ == "clean_square") {
      return square_state_ == SquareState::COMPLETE;
    } else if (task_type_ == "return_home") {
      double dx = home_x_ - current_pose_.x;
      double dy = home_y_ - current_pose_.y;
      return std::sqrt(dx*dx + dy*dy) < distance_threshold_;
    }
    return false;
  }

  int calculate_progress() {
    if (task_type_ == "return_home") {
      double initial_dist = std::sqrt((home_x_ - start_x_)*(home_x_ - start_x_) + 
                                    (home_y_ - start_y_)*(home_y_ - start_y_));
      double current_dist = std::sqrt((home_x_ - current_pose_.x)*(home_x_ - current_pose_.x) + 
                                     (home_y_ - current_pose_.y)*(home_y_ - current_pose_.y));
      if (initial_dist < 1e-6) return 100;
      int progress = static_cast<int>(100 * (1 - current_dist / initial_dist));
      return std::min(100, std::max(0, progress));
    } else if (task_type_ == "clean_square") {
      if (total_square_passes_ == 0) return 100;
      int progress = static_cast<int>(100.0 * square_passes_ / total_square_passes_);
      return std::min(100, std::max(0, progress));
    }
    return 0;
  }

  void update_distance_and_points() {
    if (first_pose_) {
      first_pose_ = false;
      return;
    }
    double dx = current_pose_.x - last_x_;
    double dy = current_pose_.y - last_y_;
    double dist = std::sqrt(dx*dx + dy*dy);
    total_distance_ += dist;
    
    if (dist > 0.01) {
      cleaned_points_ += static_cast<int>(dist * 10);
    }
    
    last_x_ = current_pose_.x;
    last_y_ = current_pose_.y;
  }

  void stop_robot() {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    publisher_->publish(twist);
  }

  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
    current_pose_ = *msg;
    pose_updated_ = true;
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CleaningActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
