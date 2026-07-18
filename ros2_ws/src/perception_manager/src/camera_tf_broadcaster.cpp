#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class CameraTfBroadcaster : public rclcpp::Node
{
public:
  CameraTfBroadcaster()
  : Node("camera_tf_broadcaster")
  {
    declare_parameter("tx", 0.0);
    declare_parameter("ty", 0.0);
    declare_parameter("tz", 0.0);
    declare_parameter("roll", 0.0);
    declare_parameter("pitch", 0.0);
    declare_parameter("yaw", 0.0);

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_transform();
  }

private:
  void publish_static_transform()
  {
    double tx = get_parameter("tx").as_double();
    double ty = get_parameter("ty").as_double();
    double tz = get_parameter("tz").as_double();
    double roll = get_parameter("roll").as_double();
    double pitch = get_parameter("pitch").as_double();
    double yaw = get_parameter("yaw").as_double();

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "camera_optical_frame";

    t.transform.translation.x = tx;
    t.transform.translation.y = ty;
    t.transform.translation.z = tz;

    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);

    t.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    t.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    t.transform.rotation.z = cr * cp * sy - sr * sp * cy;
    t.transform.rotation.w = cr * cp * cy + sr * sp * sy;

    broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
