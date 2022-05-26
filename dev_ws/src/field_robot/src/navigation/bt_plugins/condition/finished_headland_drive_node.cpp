#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "field_robot/action/condition_action.hpp"

class ActionClient : public rclcpp::Node
{
    public:
        using ConditionAction = field_robot::action::ConditionAction;
        using GoalHandleAction = rclcpp_action::ClientGoalHandle<ConditionAction>;

        explicit ActionClient(std::string name, std::string topic) : Node(name)
        {
            this->client_ptr_ = rclcpp_action::create_client<ConditionAction>(this, topic);
        }

        void send_goal()
        {
            if (!this->client_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = ConditionAction::Goal();

            RCLCPP_INFO(this->get_logger(), "Sending goal");

            auto send_goal_options = rclcpp_action::Client<ConditionAction>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&ActionClient::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback = std::bind(&ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback = std::bind(&ActionClient::result_callback, this, std::placeholders::_1);
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

    private:
        rclcpp_action::Client<ConditionAction>::SharedPtr client_ptr_;

        void goal_response_callback(std::shared_future<GoalHandleAction::SharedPtr> future)
        {

        }

        void feedback_callback(GoalHandleAction::SharedPtr, const std::shared_ptr<const ConditionAction::Feedback> feedback)
        {

        }

        void result_callback(const GoalHandleAction::WrappedResult & result)
        {

        }
};

class FinishedHeadlandDrive : public BT::AsyncActionNode
{
    public:
        FinishedHeadlandDrive(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config)
        {
            setRegistrationID("FinishedHeadlandDrive");
        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("name"),
                BT::InputPort<std::string>("topic")
            };
        }

        void halt()
        {
            _halt_requested.store(true);
        }

        BT::NodeStatus tick() override
        {
            return BT::NodeStatus::SUCCESS;
        }
    
    private:
        std::atomic_bool _halt_requested;
    
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<FinishedHeadlandDrive>("FinishedHeadlandDrive");
}