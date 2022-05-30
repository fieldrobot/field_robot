#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "field_robot/action/turn_information_action.hpp"

class ComputeGoal : public nav2_behavior_tree::BtActionNode<field_robot::action::TurnInformationAction>
{
    public:
        ComputeGoal(const std::string & name, const std::string & action_name, const BT::NodeConfiguration & conf)
            : BtActionNode<field_robot::action::TurnInformationAction>(name, action_name, conf)
        {

        }
    
        void on_tick() override
        {

        }

        BT::NodeStatus on_success()
        {
          setOutput("remaining_rows", result_.result->remaining_rows);
          setOutput("last_ron_pose", result_.result->pose);
          setOutput("turn_direction", result_.result->turn_direction);
          return BT::NodeStatus::SUCCESS;
        }

        static BT::PortsList providedPorts()
        {
          return providedBasicPorts({
            BT::OutputPort<int8>("remaining_rows"),
            BT::OutputPort<geometry_msgs::msg::Pose>("last_ron_pose"),
            BT::OutputPort<bool>("turn_direction"),
          });
        }

    private:
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder;

  builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<ComputeGoal>(name, "navigation/turn_information", config);
  };
  factory.registerBuilder<ComputeGoal>("PublishTurnInformation", builder);
}