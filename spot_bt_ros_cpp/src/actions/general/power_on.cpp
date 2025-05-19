#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/power_on.hpp"


BT::NodeStatus PowerOn::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully turned on Spot!");
    setOutput("powered_on", true);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to turn on Spot: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PowerOn::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(PowerOn, "PowerOn");