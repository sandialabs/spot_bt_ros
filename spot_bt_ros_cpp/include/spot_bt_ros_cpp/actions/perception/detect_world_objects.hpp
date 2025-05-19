#ifndef SPOT_BT_ROS_CPP__ACTIONS__PERCEPTION__DETECT_WORLD_OBJECTS_HPP_
#define SPOT_BT_ROS_CPP__ACTIONS__PERCEPTION__DETECT_WORLD_OBJECTS_HPP_

#include <vector>

#include "behaviortree_ros2/bt_service_node.hpp"
#include "bosdyn_api_msgs/msg/world_object.hpp"
#include "spot_msgs/srv/list_world_objects.hpp"

class DetectWorldObjects: public BT::RosServiceNode<spot_msgs::srv::ListWorldObjects>
{
public:
  DetectWorldObjects(const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
  : BT::RosServiceNode<spot_msgs::srv::ListWorldObjects>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::OutputPort<std::vector<bosdyn_api_msgs::msg::WorldObject>>("world_objects"),
    });
  }

  bool setRequest(Request::SharedPtr& request) override
  {
    (void)request;  // request is unused for spot_msgs::srv::ListWorldObjects
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

#endif  // SPOT_BT_ROS_CPP__ACTIONS__PERCEPTION__DETECT_WORLD_OBJECTS_HPP_