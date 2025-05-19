#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/power_off.hpp"


BT::NodeStatus PowerOff::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully powered off Spot!");
    setOutput("powered_on", false);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to power off Spot: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PowerOff::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(PowerOff, "PowerOff");