#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/estop_gentle.hpp"


BT::NodeStatus EStopGentle::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully emergency stopped (GENTLE) Spot!");
    setOutput("estopped", true);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to emergency stop (GENTLE) Spot: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus EStopGentle::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(EStopGentle, "EStopGentle");