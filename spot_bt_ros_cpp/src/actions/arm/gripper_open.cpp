#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/arm/gripper_open.hpp"

BT::NodeStatus GripperOpen::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully claimed Spot lease!");
    setOutput("gripper_open", true);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to claim Spot lease: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GripperOpen::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(GripperOpen, "GripperOpen");