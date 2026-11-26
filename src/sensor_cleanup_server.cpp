#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sist_alertatemp_volcan/action/clean_sensor_memory.hpp"

class SensorCleanupActionServer : public rclcpp::Node
{
public:
  using CleanSensorMemory = sist_alertatemp_volcan::action::CleanSensorMemory;
  using GoalHandleClean = rclcpp_action::ServerGoalHandle<CleanSensorMemory>;

  explicit SensorCleanupActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sensor_cleanup_action_server", options)
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<CleanSensorMemory>(
      this,
      "clean_sensor_memory",
      std::bind(&SensorCleanupActionServer::handle_goal, this, _1, _2),
      std::bind(&SensorCleanupActionServer::handle_cancel, this, _1),
      std::bind(&SensorCleanupActionServer::handle_accepted, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Sensor cleanup C++ action server ready.");
  }

private:
  rclcpp_action::Server<CleanSensorMemory>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CleanSensorMemory::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(
      this->get_logger(),
      "Received cleanup goal for sensor %d (clear_cache=%s)",
      goal->target_sensor_id,
      goal->clear_cache ? "true" : "false"
    );
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleClean> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_WARN(this->get_logger(), "Cancel request received (e.g., power/energy alert).");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleClean> goal_handle)
  {
    std::thread{
      std::bind(&SensorCleanupActionServer::execute, this, std::placeholders::_1),
      goal_handle
    }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleClean> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Starting diagnostics and memory cleanup...");

    auto feedback = std::make_shared<CleanSensorMemory::Feedback>();
    auto result = std::make_shared<CleanSensorMemory::Result>();

    const std::vector<std::string> stages = {
      "Running self-diagnostics",
      "Flushing volatile buffers",
      "Clearing ash telemetry cache",
      "Verifying sensor health"
    };

    const size_t total_steps = stages.size();
    float last_progress = 0.0f;

    for (size_t i = 0; i < total_steps; ++i) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cleanup canceled during stage: " + stages[i];
        result->final_progress = last_progress;
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
        return;
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));

      if (goal_handle->is_canceling()) {
        result->success = false;
        result->message = "Cleanup canceled after stage: " + stages[i];
        result->final_progress = last_progress;
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());
        return;
      }

      last_progress = static_cast<float>(i + 1) / static_cast<float>(total_steps) * 100.0f;
      feedback->progress_percent = last_progress;
      feedback->current_stage = stages[i];
      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(
        this->get_logger(),
        "Progress: %.1f%% - %s",
        feedback->progress_percent,
        feedback->current_stage.c_str()
      );
    }

    if (rclcpp::ok()) {
      result->success = true;
      result->message = "Diagnostics and memory cleanup completed.";
      result->final_progress = 100.0f;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorCleanupActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}