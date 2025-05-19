#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/arm/arm_stow.hpp"

BT::NodeStatus ArmStow::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully stowed Spot's arm!");
    setOutput("arm_stowed", true);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(logger(), "Failed to stow Spot's arm: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ArmStow::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(ArmStow, "ArmStow");