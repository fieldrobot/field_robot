#include <string>
#include <iostream>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "field_robot/action/compute_goal_action.hpp"

class ComputeGoal : public nav2_behavior_tree::BtActionNode<field_robot::action::ComputeGoalAction>
{
    public:
        //std::string namo;

        ComputeGoal(const std::string & name, const std::string & action_name, const BT::NodeConfiguration & conf)
            : BtActionNode<field_robot::action::ComputeGoalAction>(name, action_name, conf)
        {
          //this->namo = name;
        }
    
        void on_tick() override
        {
          std::string st = "GOAL COMPUTE CALLED: " + this->name();
          RCLCPP_WARN(node_->get_logger(), st);
        }

        void on_wait_for_result() override
        {
        }

        BT::NodeStatus on_success()
        {
          this->halt();
          setOutput("goal", result_.result->pose);
          std::string st = "GOAL COMPUTE FINISHED: " + this->name();
          RCLCPP_WARN(node_->get_logger(), st);
          return BT::NodeStatus::SUCCESS;
        }

        static BT::PortsList providedPorts()
        {
          return providedBasicPorts({
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
          });
        }
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder;

  builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<ComputeGoal>(name, "navigation/compute_goal_in_row", config);
  };
  factory.registerBuilder<ComputeGoal>("ComputeGoalInRow", builder);

  builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<ComputeGoal>(name, "navigation/compute_goal_headland", config);
  };
  factory.registerBuilder<ComputeGoal>("ComputeGoalHeadland", builder);

  builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<ComputeGoal>(name, "navigation/compute_goal_headland_follow", config);
  };
  factory.registerBuilder<ComputeGoal>("ComputeGoalHeadlandFollow", builder);
}