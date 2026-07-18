#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "my_interfaces/srv/set_confidence_threshold.hpp"
#include "my_interfaces/action/start_detection.hpp"
#include "my_interfaces/msg/detection.hpp"

using namespace std::placeholders;

namespace custom_action_cpp
{
class PerceptionManager : public rclcpp::Node
{
public:
  using StartDetection = my_interfaces::action::StartDetection;
  using GoalHandle = rclcpp_action::ServerGoalHandle<StartDetection>;
  using Request  = my_interfaces::srv::SetConfidenceThreshold_Request;
  using Response = my_interfaces::srv::SetConfidenceThreshold_Response;

  explicit PerceptionManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("perception_manager", options)
  {
    auto handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const StartDetection::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with %s", goal->target_class.c_str());
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [this](
      const std::shared_ptr<GoalHandle> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this](const std::shared_ptr<GoalHandle> goal_handle)
    {
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        current_goal_handle_ = goal_handle;
      }
      auto execute_in_thread = [this, goal_handle]() { this->execute(goal_handle); };
      std::thread{execute_in_thread}.detach();
    };

    this->declare_parameter<float>("confidence_threshold", 0.5f);
    detect_pub_ = this->create_publisher<my_interfaces::msg::Detection>("detections", 10);

    this->action_server_ = rclcpp_action::create_server<StartDetection>(
      this,
      "start_detection",
      handle_goal,
      handle_cancel,
      handle_accepted);

    auto handle_set_confidence = [this](
      const std::shared_ptr<Request> request,
      std::shared_ptr<Response> response)
    {
      if (request->threshold < 0.0f || request->threshold > 1.0f) {
        response->success = false;
        response->message = "threshold out of range [0.0, 1.0]";
        return;
      }
      this->set_parameter(rclcpp::Parameter("confidence_threshold", request->threshold));
      response->success = true;
      response->message = "confidence_threshold updated";
    };

    service_ = this->create_service<my_interfaces::srv::SetConfidenceThreshold>(
      "set_confidence", handle_set_confidence);
  }

private:
  rclcpp_action::Server<StartDetection>::SharedPtr action_server_;
  rclcpp::Service<my_interfaces::srv::SetConfidenceThreshold>::SharedPtr service_;
  rclcpp::Publisher<my_interfaces::msg::Detection>::SharedPtr detect_pub_;
  std::shared_ptr<GoalHandle> current_goal_handle_;
  std::mutex goal_mutex_;

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<StartDetection::Feedback>();
    auto result = std::make_shared<StartDetection::Result>();

    thread_local std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> bbox_dist(0.0f, 1.0f);

    int detections_so_far = 0;
    int loop_count = 0;

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->total_detections = detections_so_far;
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (current_goal_handle_ != goal_handle) {
          result->total_detections = detections_so_far;
          result->success = false;
          goal_handle->abort(result);
          RCLCPP_INFO(this->get_logger(), "Goal superseded by a new goal");
          return;
        }
      }

      float threshold = static_cast<float>(this->get_parameter("confidence_threshold").as_double());
      std::uniform_real_distribution<float> conf_dist(threshold, 1.0f);

      auto detection = my_interfaces::msg::Detection();
      detection.class_name = goal->target_class;
      detection.stamp = this->get_clock()->now();
      detection.confidence = conf_dist(rng);
      detection.x_min = bbox_dist(rng);
      detection.y_min = bbox_dist(rng);
      detection.x_max = detection.x_min + bbox_dist(rng) * (1.0f - detection.x_min);
      detection.y_max = detection.y_min + bbox_dist(rng) * (1.0f - detection.y_min);
      detect_pub_->publish(detection);
      ++detections_so_far;

      if (loop_count % 2 == 0) {
        feedback->current_fps = 10.0f;
        feedback->detections_so_far = detections_so_far;
        goal_handle->publish_feedback(feedback);
      }
      ++loop_count;

      loop_rate.sleep();
    }

    // Node is shutting down (Ctrl+C) while this goal is still active
    // rclcpp_action aborts the process if a goal handle is destroyed without
    // reaching a terminal state, so try to resolve it here before returning
    // The underlying action server may already be tearing down concurrently
    // with this thread, so guard against that race instead of crashing
    if (goal_handle->is_active()) {
      try {
        result->total_detections = detections_so_far;
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_INFO(this->get_logger(), "Node shutting down, aborting active goal");
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Could not resolve goal during shutdown: %s", e.what());
      }
    }
  }
};  // class PerceptionManager

}  // namespace custom_action_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<custom_action_cpp::PerceptionManager>());
  rclcpp::shutdown();
  return 0;
}
