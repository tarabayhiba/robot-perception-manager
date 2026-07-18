#include <atomic>
#include <csignal>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "my_interfaces/srv/set_confidence_threshold.hpp"
#include "my_interfaces/action/start_detection.hpp"
#include "my_interfaces/msg/detection.hpp"

namespace
{
std::atomic<bool> g_interrupt_requested{false};

void sigint_handler(int /*signum*/)
{
  g_interrupt_requested = true;
}
}  // namespace

namespace custom_action_cpp
{
class PerceptionClient : public rclcpp::Node
{
public:
  using StartDetection = my_interfaces::action::StartDetection;
  using GoalHandle = rclcpp_action::ClientGoalHandle<StartDetection>;
  using Request  = my_interfaces::srv::SetConfidenceThreshold_Request;
  using Response = my_interfaces::srv::SetConfidenceThreshold_Response;

  explicit PerceptionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("perception_action_client", options)
  {
    this->declare_parameter<std::string>("target_class", "person");

    this->client_ptr_ = rclcpp_action::create_client<StartDetection>(
      this,
      "start_detection");

    std::signal(SIGINT, sigint_handler);

    auto timer_callback_lambda = [this](){ return this->send_goal(); };
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      timer_callback_lambda);

    this->interrupt_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { this->check_interrupt(); });
  }

  void send_goal()
  {
    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = StartDetection::Goal();
    goal_msg.target_class = this->get_parameter("target_class").as_string();

    RCLCPP_INFO(this->get_logger(), "Sending goal for target_class: %s", goal_msg.target_class.c_str());

    auto send_goal_options = rclcpp_action::Client<StartDetection>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandle::SharedPtr & goal_handle)
    {
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        this->active_goal_handle_ = goal_handle;
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const StartDetection::Feedback> feedback)
    {
      RCLCPP_INFO(this->get_logger(), "fps: %.1f, detections so far: %d",
        feedback->current_fps, feedback->detections_so_far);
    };

    send_goal_options.result_callback = [this](const GoalHandle::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "done, total detections: %d, success: %d",
            result.result->total_detections, result.result->success);
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted (likely superseded by a new goal)");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_INFO(this->get_logger(), "Goal was canceled, total detections: %d",
            result.result->total_detections);
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          break;
      }
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    };
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  void check_interrupt()
  {
    if (!g_interrupt_requested.load() || cancel_sent_) {
      return;
    }
    cancel_sent_ = true;

    if (active_goal_handle_) {
      RCLCPP_INFO(this->get_logger(), "Ctrl+C received, canceling active goal");
      this->client_ptr_->async_cancel_goal(
        active_goal_handle_,
        [this](auto /*cancel_response*/) {
          RCLCPP_INFO(this->get_logger(), "Cancel request acknowledged by server");
        });
      // If the server is shutting down at the same time (everything got
      // SIGINT together via `ros2 launch`), the result may never arrive.
      // Don't wait on it forever.
      shutdown_watchdog_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() {
          if (rclcpp::ok()) {
            RCLCPP_WARN(this->get_logger(), "No result after cancel, forcing shutdown");
            rclcpp::shutdown();
          }
        });
    } else {
      RCLCPP_INFO(this->get_logger(), "Ctrl+C received, no active goal, shutting down");
      if (rclcpp::ok()) {
        rclcpp::shutdown();
      }
    }
  }

  rclcpp_action::Client<StartDetection>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr interrupt_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_watchdog_;
  GoalHandle::SharedPtr active_goal_handle_;
  bool cancel_sent_ = false;
};  // class PerceptionClient

}  // namespace custom_action_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<custom_action_cpp::PerceptionClient>());
  rclcpp::shutdown();
  return 0;
}
