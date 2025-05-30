#ifndef SPOT_BT_ROS_CPP__ACTIONS__GENERAL__POWER_OFF_HPP_
#define SPOT_BT_ROS_CPP__ACTIONS__GENERAL__POWER_OFF_HPP_

#include "behaviortree_ros2/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

class PowerOff: public BT::RosServiceNode<std_srvs::srv::Trigger>
{
public:
  PowerOff(const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
  : BT::RosServiceNode<std_srvs::srv::Trigger>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<bool>("powered_on"),
      BT::OutputPort<bool>("powered_on"),
    });
  }

  bool setRequest(Request::SharedPtr& request) override
  {
    (void)request;  // request is unused for std_srvs::srv::Trigger
    return true;
  }

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

#endif  // SPOT_BT_ROS_CPP__ACTIONS__GENERAL__POWER_OFF_HPP_