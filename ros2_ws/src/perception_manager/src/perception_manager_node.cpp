#include <memory>
#include <mutex>
#include <random>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "perception_interfaces/srv/set_confidence_threshold.hpp"
#include "perception_interfaces/action/start_detection.hpp"
#include "perception_interfaces/msg/detection.hpp"

using SetConfidenceThreshold = perception_interfaces::srv::SetConfidenceThreshold;
using StartDetection = perception_interfaces::action::StartDetection;
using GoalHandleStartDetection = rclcpp_action::ServerGoalHandle<StartDetection>;
using Detection = perception_interfaces::msg::Detection;

class PerceptionManager : public rclcpp::Node
{
public:
  PerceptionManager()
  : Node("perception_manager"), rng_(std::random_device{}())
  {
    declare_parameter("confidence_threshold", 0.5);

    detection_pub_ = create_publisher<Detection>("detections", 10);

    set_confidence_service_ = create_service<SetConfidenceThreshold>("set_confidence", std::bind(&PerceptionManager::handle_set_confidence, this, std::placeholders::_1, std::placeholders::_2));

    action_server_ = rclcpp_action::create_server<StartDetection>(
      this,
      "start_detection",
      std::bind(&PerceptionManager::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PerceptionManager::handle_cancel, this, std::placeholders::_1),
      std::bind(&PerceptionManager::handle_accepted, this, std::placeholders::_1));
  }

  ~PerceptionManager() override
  {
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      preempt_requested_ = true;
    }
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
  }

private:
  void handle_set_confidence(const std::shared_ptr<SetConfidenceThreshold::Request> request, std::shared_ptr<SetConfidenceThreshold::Response> response)
  {
    if (request->threshold < 0.0f || request->threshold > 1.0f) {
      response->success = false;
      response->message = "pick a number between 0.0 and 1.0 please!";
      return;
    }

    set_parameter(rclcpp::Parameter("confidence_threshold", static_cast<double>(request->threshold)));
    response->success = true;
    response->message = "confidence_threshold was succesfully updated";
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const StartDetection::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received goal for target_class='%s'", goal->target_class.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleStartDetection> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleStartDetection> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      preempt_requested_ = true;
    }
    if (execution_thread_.joinable()) {
      execution_thread_.join();
    }
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      preempt_requested_ = false;
    }

    execution_thread_ = std::thread(std::bind(&PerceptionManager::execute, this, goal_handle));
  }

  void execute(const std::shared_ptr<GoalHandleStartDetection> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<StartDetection::Feedback>();
    auto result = std::make_shared<StartDetection::Result>();

    int detections_so_far = 0;
    rclcpp::Rate loop_rate(10.0);
    auto last_feedback_time = now();

    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (preempt_requested_) {
          // Preemption is server-initiated, so the goal was never put in the CANCELING
          // state by a client request; canceled() would throw here. Just drop our
          // reference to goal_handle: its destructor auto-transitions it to CANCELED.
          RCLCPP_INFO(get_logger(), "Goal preempted by a new goal");
          return;
        }
      }

      if (goal_handle->is_canceling()) {
        result->total_detections = detections_so_far;
        result->success = true;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return;
      }

      publish_detection(goal->target_class);
      detections_so_far++;

      if ((now() - last_feedback_time).seconds() >= 0.2) {
        feedback->current_fps = 10.0f;
        feedback->detections_so_far = detections_so_far;
        goal_handle->publish_feedback(feedback);
        last_feedback_time = now();
      }

      loop_rate.sleep();
    }
  }

  void publish_detection(const std::string & target_class)
  {
    double threshold = get_parameter("confidence_threshold").as_double();

    std::uniform_real_distribution<float> conf_dist(static_cast<float>(threshold), 1.0f);
    std::uniform_real_distribution<float> x_dist(0.0f, 600.0f);
    std::uniform_real_distribution<float> y_dist(0.0f, 440.0f);
    std::uniform_real_distribution<float> size_dist(10.0f, 40.0f);

    Detection msg;
    msg.class_name = target_class;
    msg.confidence = conf_dist(rng_);
    msg.x_min = x_dist(rng_);
    msg.y_min = y_dist(rng_);
    msg.x_max = msg.x_min + size_dist(rng_);
    msg.y_max = msg.y_min + size_dist(rng_);
    msg.stamp = now();

    detection_pub_->publish(msg);
  }

  rclcpp::Publisher<Detection>::SharedPtr detection_pub_;
  rclcpp::Service<SetConfidenceThreshold>::SharedPtr set_confidence_service_;
  rclcpp_action::Server<StartDetection>::SharedPtr action_server_;

  std::thread execution_thread_;
  std::mutex goal_mutex_;
  bool preempt_requested_ = false;
  std::mt19937 rng_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionManager>());
  rclcpp::shutdown();
  return 0;
}
