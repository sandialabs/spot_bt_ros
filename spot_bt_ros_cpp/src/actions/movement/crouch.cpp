#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/movement/crouch.hpp"


BT::NodeStatus Crouch::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Spot successfully crouched!");
    setOutput("crouching", true);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Spot failed to crouch: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Crouch::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(Crouch, "Crouch");