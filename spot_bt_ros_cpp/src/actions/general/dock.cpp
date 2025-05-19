#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/general/dock.hpp"


bool Dock::setRequest(Request::SharedPtr& request)
{
  getInput("dock_id", request->dock_id);
  return true;
}

BT::NodeStatus Dock::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->success) {
    RCLCPP_INFO(logger(), "Successfully docked Spot!");
    setOutput("docked", true);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to dock Spot: %s", response->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus Dock::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(Dock, "Dock");