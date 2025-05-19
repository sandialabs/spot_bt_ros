#ifndef SPOT_BT_ROS_CPP__ACTIONS__PLANNER__COMPUTE_NEW_WAYPOINT_HPP_
#define SPOT_BT_ROS_CPP__ACTIONS__PLANNER__COMPUTE_NEW_WAYPOINT_HPP_

#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "spot_bt_ros_msgs/action/plan_spot_body_task.hpp"

class ComputeNewWaypoint: public BT::RosActionNode<spot_bt_ros_msgs::action::PlanSpotBodyTask>
{
public:
  ComputeNewWaypoint(const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
  : BT::RosActionNode<spot_bt_ros_msgs::action::PlanSpotBodyTask>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("target"),
    });
  }

  bool setGoal(RosActionNode::Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const RosActionNode::WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};

#endif  // SPOT_BT_ROS_CPP__ACTIONS__PLANNER__COMPUTE_NEW_WAYPOINT_HPP_