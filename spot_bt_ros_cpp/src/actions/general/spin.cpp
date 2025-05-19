#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/spin.hpp"


BT::NodeStatus Spin::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Spot successfully standing!");
    setOutput("standing", true);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Spot failed to stand: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Spin::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(Spin, "Spin");