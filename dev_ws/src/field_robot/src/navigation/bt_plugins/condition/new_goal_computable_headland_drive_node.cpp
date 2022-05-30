#include "nav2_behavior_tree/bt_action_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "field_robot/action/robot_pose_update_action.hpp"

class GoalComputableNode : public nav2_behavior_tree::BtActionNode<field_robot::action::RobotPoseUpdateAction>
{
    public:
        GoalComputableNode(const std::string & name, const std::string & action_name, const BT::NodeConfiguration & conf)
            : BtActionNode<field_robot::action::RobotPoseUpdateAction>(name, action_name, conf)
        {

        }
    
        void on_tick() override
        {

        }

        BT::NodeStatus on_success()
        {
          setOutput("last_ron_pose", result_.result->pose);
          int rows = !getInput<int>("remaining_rows");
          setOutput("remaining_rows", rows-1);
          return BT::NodeStatus::SUCCESS;
        }

        static BT::PortsList providedPorts()
        {
            return providedBasicPorts({
                BT::InputPort<int>("remaining_rows"),
                BT::OutputPort<geometry_msgs::msg::Pose>("last_ron_pose"),
                BT::OutputPort<int>("remaining_rows"),
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
    return std::make_unique<GoalComputableNode>(name, "navigation/new_goal_computable_headland_drive", config);
  };
  factory.registerBuilder<GoalComputableNode>("NewGoalComputable", builder);
}