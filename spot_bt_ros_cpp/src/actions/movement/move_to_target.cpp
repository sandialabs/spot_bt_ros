#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/movement/move_to_target.hpp"

bool MoveToTarget::setGoal(RosActionNode::Goal& goal)
{
  goal.header.stamp = now();
  goal.action = "move_to_target";
  getInput("target", goal.goal);
  return true;
}

BT::NodeStatus MoveToTarget::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  if (wr.result->success) {
    RCLCPP_INFO(logger(), "Successfully moved to target!");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(logger(), "Failed to move to target: %s ", wr.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveToTarget::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(MoveToTarget, "MoveToTarget");