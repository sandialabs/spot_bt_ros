#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/release_lease.hpp"


BT::NodeStatus ReleaseLease::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully released Spot lease!");
    setOutput("lease_claimed", false);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to release Spot lease: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ReleaseLease::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(ReleaseLease, "ReleaseLease");