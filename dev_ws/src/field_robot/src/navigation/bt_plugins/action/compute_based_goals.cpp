#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "field_robot/action/compute_based_goal_action.hpp"

class ComputeBasedGoal : public nav2_behavior_tree::BtActionNode<field_robot::action::ComputeBasedGoalAction>
{
    public:
        ComputeBasedGoal(const std::string & name, const std::string & action_name, const BT::NodeConfiguration & conf)
            : BtActionNode<field_robot::action::ComputeBasedGoalAction>(name, action_name, conf)
        {

        }
    
        void on_tick() override
        {
            getInput("remaining_rows", goal_.remaining_rows);
            getInput("last_ron_pose", goal_.pose);
            getInput("turn_direction", goal_.turn_direction);
        }

        BT::NodeStatus on_success()
        {
          setOutput("goal", result_.result->pose);
          return BT::NodeStatus::SUCCESS;
        }

        static BT::PortsList providedPorts()
        {
          return providedBasicPorts({
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
            BT::InputPort<int>("remaining_rows"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("last_ron_pose"),
            BT::InputPort<bool>("turn_direction"),
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
    return std::make_unique<ComputeBasedGoal>(name, "navigation/compute_goal/leaving_row", config);
  };
  factory.registerBuilder<ComputeBasedGoal>("ComputeBasedGoalLeavingRow", builder);

  builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<ComputeBasedGoal>(name, "navigation/compute_goal/out_of_row", config);
  };
  factory.registerBuilder<ComputeBasedGoal>("ComputeBasedGoalOutOfRow", builder);
}