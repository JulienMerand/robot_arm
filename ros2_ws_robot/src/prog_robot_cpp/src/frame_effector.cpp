#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class Frame_Effector : public rclcpp::Node
{
public:
  Frame_Effector()
  : Node("frame_effector")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&Frame_Effector::broadcast_timer_callback, this));
  }

private:
  void broadcast_timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "link6_1";
    t.child_frame_id = "frame_effector";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.130;
    // t.transform.rotation.x = 0.0;
    // t.transform.rotation.y = 0.0;
    // t.transform.rotation.z = 0.0;
    // t.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t);
  }

rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Frame_Effector>());
  rclcpp::shutdown();
  return 0;
}