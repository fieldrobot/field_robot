#include <string>
#include <iostream>

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
          RCLCPP_WARN(node_->get_logger(), "Action Server should be called!");
        }

        /*BT::NodeStatus tick() override
        {
          RCLCPP_WARN(node_->get_logger(), "WARNING IMPORTANT WARNING IMPORTANT WARNING IMPORTANT");
          if (this->succeeded == true)
          {
            setStatus(BT::NodeStatus::IDLE);
          }
          

          switch (status()) {
            case BT::NodeStatus::SUCCESS:
              RCLCPP_WARN(node_->get_logger(), "Action Server XXX SSSSSSSUUUUUUu!");
              break;
            case BT::NodeStatus::RUNNING:
              RCLCPP_WARN(node_->get_logger(), "Action Server XXX RRRRRRRRRUUUUUUUU!");
              break;
            case BT::NodeStatus::FAILURE:
              RCLCPP_WARN(node_->get_logger(), "Action Server XXX FFFFFFFFAAAAAAA!");
              break;
            case BT::NodeStatus::IDLE:
              RCLCPP_WARN(node_->get_logger(), "Action Server XXX IIIIIIIIIIIDDDDDDDDD!");
              break;
            default:
              RCLCPP_WARN(node_->get_logger(), "Action Server XXX DDDDDDDDDDDEEEEEEEEE!");
          }
          // first step to be done only at the beginning of the Action
          if (status() == BT::NodeStatus::IDLE) {
            // setting the status to RUNNING to notify the BT Loggers (if any)
            setStatus(BT::NodeStatus::RUNNING);

            // user defined callback
            on_tick();

            on_new_goal_received();
          }

          // The following code corresponds to the "RUNNING" loop
          if (rclcpp::ok() && !goal_result_available_) {
            // user defined callback. May modify the value of "goal_updated_"
            on_wait_for_result();

            auto goal_status = goal_handle_->get_status();
            if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
              goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
            {
              goal_updated_ = false;
              on_new_goal_received();
            }

            rclcpp::spin_some(node_);

            // check if, after invoking spin_some(), we finally received the result
            if (!goal_result_available_) {
              // Yield this Action, returning RUNNING
              return BT::NodeStatus::RUNNING;
            }
          }

          switch (result_.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              return on_success();

            case rclcpp_action::ResultCode::ABORTED:
              return on_aborted();

            case rclcpp_action::ResultCode::CANCELED:
              return on_cancelled();

            default:
              throw std::logic_error("BtActionNode::Tick: invalid status value");
          }
        }*/

        void on_wait_for_result() override
        {
          RCLCPP_WARN(node_->get_logger(), "Action Server waits for result!");
        }

        BT::NodeStatus on_success()
        {
          this->halt();
          setOutput("goal", result_.result->pose);
          //succeeded = true;
          return BT::NodeStatus::SUCCESS;
        }

        static BT::PortsList providedPorts()
        {
          return providedBasicPorts({
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
          });
        }

    private:
      //bool succeeded = false;
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
}