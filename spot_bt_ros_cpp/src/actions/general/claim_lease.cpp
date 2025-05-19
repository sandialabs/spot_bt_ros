#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/claim_lease.hpp"


BT::NodeStatus ClaimLease::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully claimed Spot lease!");
    setOutput("lease_claimed", true);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to claim Spot lease: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ClaimLease::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(ClaimLease, "ClaimLease");