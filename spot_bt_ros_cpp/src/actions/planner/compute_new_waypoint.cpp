#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/planner/compute_new_waypoint.hpp"

bool ComputeNewWaypoint::setGoal(RosActionNode::Goal& goal)
{
  goal.header.stamp = now();
  goal.action = "search";
  return true;
}

BT::NodeStatus ComputeNewWaypoint::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  if (wr.result->success) {
    RCLCPP_INFO(logger(), "Successfully computed new waypoint!");
    setOutput("target", wr.result->goal_pose);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(logger(), "Failed to compute new waypoint: %s ", wr.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputeNewWaypoint::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(ComputeNewWaypoint, "ComputeNewWaypoint");