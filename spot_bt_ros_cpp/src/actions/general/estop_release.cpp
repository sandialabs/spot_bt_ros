#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/estop_release.hpp"


BT::NodeStatus EStopRelease::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully released emergency stop for Spot!");
    setOutput("estopped", false);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to release emergency stop for Spot: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus EStopRelease::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(EStopRelease, "EStopRelease");