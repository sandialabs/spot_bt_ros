#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/wait.hpp"

bool Wait::setGoal(RosActionNode::Goal& goal)
{
  goal.header.stamp = now();
  goal.action = "wait";
  getInput("wait_duration", goal.options[0]);
  return true;
}

BT::NodeStatus Wait::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  if (wr.result->success) {
    RCLCPP_INFO(logger(), "Successfully waited!");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(logger(), "Failed to wait: %s ", wr.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Wait::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(Wait, "Wait");