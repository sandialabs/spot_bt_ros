#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/movement/sit.hpp"


BT::NodeStatus Sit::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Spot successfully sat!");
    setOutput("standing", false);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Spot failed to sit: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Sit::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(Sit, "Sit");