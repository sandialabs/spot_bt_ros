#include "behaviortree_ros2/plugins.hpp"
#include "bosdyn_api_msgs/msg/world_object_type.hpp"
#include "spot_bt_ros_cpp/actions/perception/detect_fiducial_markers.hpp"


bool DetectFiducialMarkers::setRequest(Request::SharedPtr& request)
{
  // getInput("dock_id", request->dock_id);
  auto msg = bosdyn_api_msgs::msg::WorldObjectType();
  msg.value = 2;  // specific value for world_object_pb2.WORLD_OBJECT_APRILTAG = Literal[2]
  std::vector<bosdyn_api_msgs::msg::WorldObjectType> desired_objects = {msg};
  request->request.object_type = desired_objects;
  return true;
}

BT::NodeStatus DetectFiducialMarkers::onResponseReceived(const Response::SharedPtr& response)
{
  if (response->response.world_objects.size() > 0) {
    RCLCPP_INFO(logger(), "Successfully detected fiducial markers!");
    setOutput("fiducials", response->response.world_objects);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(logger(), "Failed to detect fiducial markers!");
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DetectFiducialMarkers::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(DetectFiducialMarkers, "DetectFiducialMarkers");