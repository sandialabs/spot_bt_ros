#ifndef SPOT_BT_ROS_CPP__ACTIONS__ARM__ARM_FREEZE_HPP_
#define SPOT_BT_ROS_CPP__ACTIONS__ARM__ARM_FREEZE_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "spot_bt_ros_msgs/action/spot_arm_task.hpp"

class ArmFreeze: public BT::RosActionNode<spot_bt_ros_msgs::action::SpotArmTask>
{
public:
  ArmFreeze(const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
  : BT::RosActionNode<spot_bt_ros_msgs::action::SpotArmTask>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    // TODO Change ports for this one.
    return providedBasicPorts({
      BT::InputPort<bool>("gripper_open"),
      BT::OutputPort<bool>("gripper_open"),
    });
  }

  bool setGoal(RosActionNode::Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const RosActionNode::WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};

#endif  // SPOT_BT_ROS_CPP__ACTIONS__ARM__ARM_FREEZE_HPP_