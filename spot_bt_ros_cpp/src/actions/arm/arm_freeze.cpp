#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/arm/arm_freeze.hpp"

bool ArmFreeze::setGoal(RosActionNode::Goal& goal)
{
  goal.header.stamp = now();
  goal.action = "freeze";
  return true;
}

BT::NodeStatus ArmFreeze::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  if (wr.result->success) {
    RCLCPP_INFO(logger(), "Successfully froze Spot's arm!");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(logger(), "Failed to freeze Spot's arm: %s ", wr.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ArmFreeze::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(ArmFreeze, "ArmFreeze");