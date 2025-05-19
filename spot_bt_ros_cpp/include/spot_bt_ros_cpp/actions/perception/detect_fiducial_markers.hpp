#ifndef SPOT_BT_ROS_CPP__ACTIONS__PERCEPTION__DETECT_FIDUCIAL_MARKERS_HPP_
#define SPOT_BT_ROS_CPP__ACTIONS__PERCEPTION__DETECT_FIDUCIAL_MARKERS_HPP_

#include <vector>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "bosdyn_api_msgs/msg/world_object.hpp"
#include "spot_msgs/srv/list_world_objects.hpp"

class DetectFiducialMarkers: public BT::RosServiceNode<spot_msgs::srv::ListWorldObjects>
{
public:
  DetectFiducialMarkers(const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
  : BT::RosServiceNode<spot_msgs::srv::ListWorldObjects>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<int>("dock_id"),
      BT::OutputPort<std::vector<bosdyn_api_msgs::msg::WorldObject>>("fiducials"),
    });
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

#endif  // SPOT_BT_ROS_CPP__ACTIONS__PERCEPTION__DETECT_FIDUCIAL_MARKERS_HPP_