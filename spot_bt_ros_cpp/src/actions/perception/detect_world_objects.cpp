#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/perception/detect_world_objects.hpp"

BT::NodeStatus DetectWorldObjects::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->response.world_objects.size() > 0) {
    RCLCPP_INFO(logger(), "Successfully detected world objects!");
    setOutput("fiducials", response->response.world_objects);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to detect world objects!");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DetectWorldObjects::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(DetectWorldObjects, "DetectWorldObjects");