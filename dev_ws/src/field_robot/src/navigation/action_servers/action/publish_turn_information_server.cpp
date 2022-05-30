#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "field_robot/action/turn_information_action.hpp"

class PublishTurnInformationActionServer : public rclcpp::Node
{
    public:
        using TurnInformationAction = field_robot::action::TurnInformationAction;
        using GoalHandleTurnInformationAction = rclcpp_action::ServerGoalHandle<TurnInformationAction>;

        PublishTurnInformationActionServer() : Node("publish_turn_information_action_server")
        {

            this->declare_parameter("action_topic", "front_empty");
            this->get_parameter("action_topic", action_topic);
            this->declare_parameter("robot_frame", "base_footprint");
            this->get_parameter("robot_frame", robot_frame);
            
            action_server_ = rclcpp_action::create_server<TurnInformationAction>(
                this,
                action_topic,
                std::bind(&PublishTurnInformationActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&PublishTurnInformationActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&PublishTurnInformationActionServer::handle_accepted, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp_action::Server<TurnInformationAction>::SharedPtr action_server_;

        bool next_direction = false; // true means right, false means left
        int8_t next_row_amount = 1;

        std::string action_topic;
        std::string robot_frame;
        
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TurnInformationAction::Goal> goal)
        {
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleTurnInformationAction> goal_handle)
        {
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleTurnInformationAction> goal_handle)
        {
            std::thread{std::bind(&PublishTurnInformationActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleTurnInformationAction> goal_handle)
        {
            auto result = std::make_shared<TurnInformationAction::Result>();

            result->remaining_rows = next_row_amount;
            result->turn_direction = next_direction;
            result->pose.header.frame_id = robot_frame;
            result->pose.header.stamp = this->get_clock()->now();
            result->pose.pose.position.x = 0.0;
            result->pose.pose.position.y = 0.0;
            result->pose.pose.position.z = 0.0;
            result->pose.pose.orientation.x = 0.0;
            result->pose.pose.orientation.y = 0.0;
            result->pose.pose.orientation.z = 0.0;
            result->pose.pose.orientation.w = 1.0;

            next_direction = !next_direction;
            next_row_amount++;

            goal_handle->succeed(result);
            return;
        }

};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublishTurnInformationActionServer>());
    rclcpp::shutdown();
    return 0;
}
