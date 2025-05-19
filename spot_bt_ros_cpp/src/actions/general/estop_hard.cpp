#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/estop_hard.hpp"


BT::NodeStatus EStopHard::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully emergency stopped (HARD) Spot!");
    setOutput("estopped", true);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to emergency stop (HARD) Spot: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus EStopHard::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(EStopHard, "EStopHard");