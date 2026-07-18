#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "perception_interfaces/srv/set_confidence_threshold.hpp"

using SetConfidenceThreshold = perception_interfaces::srv::SetConfidenceThreshold;

class PerceptionManager : public rclcpp::Node
{
public:
  PerceptionManager() : Node("perception_manager")
  {
    declare_parameter("confidence_threshold", 0.5);
    set_confidence_service_ = create_service<SetConfidenceThreshold>("set_confidence", std::bind(&PerceptionManager::handle_set_confidence, this, std::placeholders::_1, std::placeholders::_2));
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

  rclcpp::Service<SetConfidenceThreshold>::SharedPtr set_confidence_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionManager>());
  rclcpp::shutdown();
  return 0;
}
