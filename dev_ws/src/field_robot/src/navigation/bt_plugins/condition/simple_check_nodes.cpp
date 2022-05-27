#include "nav2_behavior_tree/bt_action_node.hpp"
#include "field_robot/action/condition_action.hpp"

class SimpleCheckNode : public nav2_behavior_tree::BtActionNode<field_robot::action::ConditionAction>
{
    public:
        SimpleCheckNode(const std::string & name, const std::string & action_name, const BT::NodeConfiguration & conf)
            : BtActionNode<field_robot::action::ConditionAction>(name, action_name, conf)
        {

        }
    
        void on_tick() override
        {

        }

        BT::NodeStatus on_success()
        {
          return BT::NodeStatus::SUCCESS;
        }

        static BT::PortsList providedPorts()
        {
            return {};
        }

    private:
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder;

  builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<SimpleCheckNode>(name, "navigation/front_empty", config);
  };
  factory.registerBuilder<SimpleCheckNode>("FrontEmpty", builder);

  builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<SimpleCheckNode>(name, "navigation/working", config);
  };
  factory.registerBuilder<SimpleCheckNode>("Working", builder);

  builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<SimpleCheckNode>(name, "navigation/completed_leaving_row", config);
  };
  factory.registerBuilder<SimpleCheckNode>("CompletedLeavingRow", builder);

  builder = [](const std::string & name, const BT::NodeConfiguration & config)
  {
    return std::make_unique<SimpleCheckNode>(name, "navigation/finished_headland_drive", config);
  };
  factory.registerBuilder<SimpleCheckNode>("FinishedHeadlandDrive", builder);
}