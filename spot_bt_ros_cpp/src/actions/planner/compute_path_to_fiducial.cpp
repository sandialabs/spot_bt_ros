#include "behaviortree_ros2/plugins.hpp"
#include "spot_bt_ros_cpp/actions/planner/compute_path_to_fiducial.hpp"

bool ComputePathToFiducial::setGoal(RosActionNode::Goal& goal)
{
  // Select the proper fiducial value
  getInput("dock_id", dock_id_);
  getInput("fiducials", fiducials_);
  RCLCPP_INFO(logger(), "Length of saved fiducials: %d", static_cast<int>(fiducials_.size()));
  bosdyn_api_msgs::msg::WorldObject select_fiducial;
  if (fiducials_.size() >= 1) {
    for (const bosdyn_api_msgs::msg::WorldObject& fiducial : fiducials_) {
      if (fiducial.apriltag_properties.tag_id != dock_id_) {
        select_fiducial = fiducial;
      }
    }
  }

  // Create the goal
  goal.header.stamp = now();
  goal.action = "fiducial";
  goal.world_object = select_fiducial;
  return true;
}

BT::NodeStatus ComputePathToFiducial::onResultReceived(const RosActionNode::WrappedResult& wr)
{
  if (wr.result->success) {
    RCLCPP_INFO(logger(), "Successfully computed path to fiducial!");
    setOutput("target", wr.result->goal_pose);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_ERROR(logger(), "Failed to move to fiducial: %s ", wr.result->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePathToFiducial::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error));
  return BT::NodeStatus::FAILURE;
}

// Plugin registration
CreateRosNodePlugin(ComputePathToFiducial, "ComputePathToFiducial");