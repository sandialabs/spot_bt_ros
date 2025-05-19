#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/planner/compute_path_to_pose.hpp"

bool ComputePathToPose::setGoal(RosActionNode::Goal& goal)
{
  goal.header.stamp = now();
  goal.action = "target";
  getInput("target", goal.target);
  return true;
}

BT::NodeStatus ComputePathToPose::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  if (wr.result->success) {
    RCLCPP_INFO(logger(), "Successfully computed path to target pose!");
    setOutput("target", wr.result->goal_pose);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(logger(), "Failed to compute path to target pose: %s ", wr.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathToPose::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(ComputePathToPose, "ComputePathToPose");