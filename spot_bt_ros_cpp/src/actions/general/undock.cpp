#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/undock.hpp"


BT::NodeStatus Undock::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully undocked Spot!");
    setOutput("docked", false);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to undock Spot: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Undock::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(Undock, "Undock");