#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/arm/arm_unstow.hpp"

BT::NodeStatus ArmUnstow::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully unstowed Spot's arm!");
    setOutput("arm_stowed", false);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(logger(), "Failed to unstow Spot's arm: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ArmUnstow::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(ArmUnstow, "ArmUnstow");