#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/arm/arm_carry.hpp"

BT::NodeStatus ArmCarry::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully carrying!");
    setOutput("arm_carrying", true);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to carry: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ArmCarry::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(ArmCarry, "ArmCarry");