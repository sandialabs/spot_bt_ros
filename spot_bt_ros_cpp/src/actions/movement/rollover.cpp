#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "std_srvs/srv/trigger.hpp"

class Rollover: public BT::RosServiceNode<std_srvs::srv::Trigger>
{
  public:
    Rollover(const std::string& name,
      const BT::NodeConfig& conf,
      const BT::RosNodeParams& params)
    : BT::RosServiceNode<std_srvs::srv::Trigger>(name, conf, params)
    {}

    // TODO: Look into if I need to override this...
    bool setRequest(Request::SharedPtr& request) override
    {
      (void)request;  // request is unused for std_srvs::srv::Trigger
      return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
      if (response->success) {
        RCLCPP_INFO(logger(), "Spot successfully rolled over!");
        // setOutput("crouching", true);
        return BT::NodeStatus::SUCCESS;
      }

      RCLCPP_INFO(logger(), "Spot failed to roll over: %s", response->message.c_str());
      return BT::NodeStatus::FAILURE;
    }

    virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override
    {
      RCLCPP_ERROR(logger(), "Error: %d", error);
      return BT::NodeStatus::FAILURE;
    }
};

// Plugin registration
CreateRosNodePlugin(Rollover, "Rollover");