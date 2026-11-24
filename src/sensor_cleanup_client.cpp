#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sist_alertatemp_volcan/action/clean_sensor_memory.hpp"

namespace
{
std::atomic<bool> g_cancel_requested{false};

void sigint_handler(int signal)
{
  (void)signal;
  g_cancel_requested.store(true);
}
}  // namespace

class SensorCleanupActionClient : public rclcpp::Node
{
public:
  using CleanSensorMemory = sist_alertatemp_volcan::action::CleanSensorMemory;
  using GoalHandleClean = rclcpp_action::ClientGoalHandle<CleanSensorMemory>;

  explicit SensorCleanupActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sensor_cleanup_action_client", options), goal_done_(false)
  {
    using namespace std::placeholders;

    client_ptr_ = rclcpp_action::create_client<CleanSensorMemory>(
      this,
      "clean_sensor_memory"
    );

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SensorCleanupActionClient::timer_callback, this)
    );
  }

  bool is_goal_done() const
  {
    return goal_done_.load();
  }

  void send_goal(int32_t sensor_id, bool clear_cache)
  {
    using namespace std::placeholders;
    goal_done_.store(false);

    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
      goal_done_.store(true);
      return;
    }

    auto goal_msg = CleanSensorMemory::Goal();
    goal_msg.target_sensor_id = sensor_id;
    goal_msg.clear_cache = clear_cache;

    RCLCPP_INFO(this->get_logger(), "Sending cleanup goal for sensor %d...", sensor_id);

    rclcpp_action::Client<CleanSensorMemory>::SendGoalOptions send_goal_options;
    send_goal_options.goal_response_callback =
      std::bind(&SensorCleanupActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&SensorCleanupActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&SensorCleanupActionClient::result_callback, this, _1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<CleanSensorMemory>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  GoalHandleClean::SharedPtr goal_handle_;
  std::atomic<bool> goal_done_;
  bool cancel_sent_{false};

  void timer_callback()
  {
    if (g_cancel_requested.load() && !cancel_sent_ && goal_handle_ != nullptr && !goal_done_.load()) {
      cancel_sent_ = true;
      RCLCPP_WARN(this->get_logger(), "Requesting goal cancellation...");
      client_ptr_->async_cancel_goal(goal_handle_);
    }
  }

  void goal_response_callback(const GoalHandleClean::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
      goal_done_.store(true);
    } else {
      goal_handle_ = goal_handle;
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server.");
    }
  }

  void feedback_callback(
    GoalHandleClean::SharedPtr,
    const std::shared_ptr<const CleanSensorMemory::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[%.1f%%] Stage: %s",
      feedback->progress_percent,
      feedback->current_stage.c_str()
    );
  }

  void result_callback(const GoalHandleClean::WrappedResult & result)
  {
    goal_done_.store(true);

    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted.");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(
          this->get_logger(),
          "Goal canceled. Final progress: %.1f%% | Msg: %s",
          result.result->final_progress,
          result.result->message.c_str()
        );
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
        return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Completed -> success=%s, progress=%.1f%%, msg=\"%s\"",
      result.result->success ? "true" : "false",
      result.result->final_progress,
      result.result->message.c_str()
    );
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, sigint_handler);

  auto client_node = std::make_shared<SensorCleanupActionClient>();
  client_node->send_goal(1, true);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);

  while (rclcpp::ok() && !client_node->is_goal_done()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  rclcpp::shutdown();
  return 0;
}