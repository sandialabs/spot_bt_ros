#ifndef SPOT_BT_ROS_CPP__ACTIONS__PLANNER__COMPUTE_PATH_TO_FIDUCIAL_HPP_
#define SPOT_BT_ROS_CPP__ACTIONS__PLANNER__COMPUTE_PATH_TO_FIDUCIAL_HPP_

#include <vector>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "bosdyn_api_msgs/msg/world_object.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "spot_bt_ros_msgs/action/plan_spot_body_task.hpp"

class ComputePathToFiducial: public BT::RosActionNode<spot_bt_ros_msgs::action::PlanSpotBodyTask>
{
public:
  ComputePathToFiducial(const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
  : BT::RosActionNode<spot_bt_ros_msgs::action::PlanSpotBodyTask>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<int>("dock_id"),
      BT::InputPort<std::vector<bosdyn_api_msgs::msg::WorldObject>>("fiducials"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("target"),
    });
  }

  bool setGoal(RosActionNode::Goal& goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const RosActionNode::WrappedResult& wr) override;

  virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;

private:
  int dock_id_;
  std::vector<bosdyn_api_msgs::msg::WorldObject> fiducials_;
};

#endif  // SPOT_BT_ROS_CPP__ACTIONS__PLANNER__COMPUTE_PATH_TO_FIDUCIAL_HPP_