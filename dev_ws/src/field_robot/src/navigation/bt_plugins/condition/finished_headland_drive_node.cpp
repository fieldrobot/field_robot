#include "behaviortree_cpp_v3/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "field_robot/action/condition_action.hpp"

class ActionClient : public rclcpp::Node
{
    public:
        using ConditionAction = field_robot::action::ConditionAction;
        using GoalHandleAction = rclcpp_action::ClientGoalHandle<ConditionAction>;
        using CancelResponse = typename ConditionAction::Impl::CancelGoalService::Response;

        bool is_goal_active = false;
        bool cancelable = false;
        bool cancelled = false;
        BT::NodeStatus status = BT::NodeStatus::IDLE;

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
            this->is_goal_active = true;
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

        void cancel_goal()
        {
            if (!this->is_goal_active && this->cancelable) {
                this->client_ptr_->async_cancel_goal(goal_handle_ptr_, std::bind(&ActionClient::cancelled_callback, this, std::placeholders::_1));
                this->cancelled = true;
            }
        }

    private:
        rclcpp_action::Client<ConditionAction>::SharedPtr client_ptr_;
        GoalHandleAction::SharedPtr goal_handle_ptr_;

        void goal_response_callback(std::shared_future<GoalHandleAction::SharedPtr> future)
        {
            goal_handle_ptr_ = future.get();
            if(!goal_handle_ptr_) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

        void feedback_callback(GoalHandleAction::SharedPtr ghasptr, const std::shared_ptr<const ConditionAction::Feedback> feedback)
        {
            goal_handle_ptr_ = ghasptr;
            this->cancelable = true;
        }

        void result_callback(const GoalHandleAction::WrappedResult & result)
        {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    this->status = BT::NodeStatus::SUCCESS;
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    this->status = BT::NodeStatus::FAILURE;
                    return;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                    this->status = BT::NodeStatus::SUCCESS;
                    return;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    return;
            }

            this->is_goal_active = false;
            this->cancelable = false;
            this->cancelled = false;
        }

        void cancelled_callback(CancelResponse::SharedPtr cancel_response)
        {
            this->is_goal_active = false;
            this->cancelled = false;
            this->cancelable = false;
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
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
            _goal_send.store(false);
            switch (_goal_send)
            {
                case false:
                    /* code */
                    break;
                
                case true:
                    break;
            }
            return BT::NodeStatus::RUNNING;
        }
    
    private:
        
        std::atomic_bool _halt_requested;
        std::atomic_bool _goal_send;
    
};

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<FinishedHeadlandDrive>("FinishedHeadlandDrive");
}