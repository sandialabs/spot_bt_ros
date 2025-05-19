#ifndef SPOT_BT_ROS_CPP__ACTIONS__GENERAL__DOCK_HPP_
#define SPOT_BT_ROS_CPP__ACTIONS__GENERAL__DOCK_HPP_

#include "behaviortree_ros2/bt_service_node.hpp"
#include "spot_msgs/srv/dock.hpp"

class Dock: public BT::RosServiceNode<spot_msgs::srv::Dock>
{
public:
  Dock(const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
  : BT::RosServiceNode<spot_msgs::srv::Dock>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<int>("dock_id"),
      BT::InputPort<bool>("docked"),
      BT::OutputPort<bool>("docked"),
    });
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

#endif  // SPOT_BT_ROS_CPP__ACTIONS__GENERAL__DOCK_HPP_