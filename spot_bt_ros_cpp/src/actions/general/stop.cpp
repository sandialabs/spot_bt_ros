#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/stop.hpp"


BT::NodeStatus Stop::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully stopped Spot in place!");
    setOutput("stopped", true);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to stop Spot in place: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Stop::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(Stop, "Stop");