#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/arm/gripper_close.hpp"

BT::NodeStatus GripperClose::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully closed Spot's gripper!");
    setOutput("gripper_open", false);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to close Spot's gripper: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GripperClose::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(GripperClose, "GripperClose");