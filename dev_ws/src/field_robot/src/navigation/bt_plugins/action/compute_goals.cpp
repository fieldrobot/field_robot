#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "field_robot/action/compute_goal_action.hpp"

class ComputeGoal : public nav2_behavior_tree::BtActionNode<field_robot::action::ComputeGoalAction>
{
    public:
        ComputeGoal(const std::string & name, const std::string & action_name, const BT::NodeConfiguration & conf)
            : BtActionNode<field_robot::action::ComputeGoalAction>(name, action_name, conf)
        {

        }
    
        void on_tick() override
        {

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
    return std::make_unique<ComputeGoal>(name, "navigation/compute_goal/in_row", config);
  };
  factory.registerBuilder<ComputeGoal>("ComputeGoalInRow", builder);
}