#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "field_robot/action/compute_based_goal_action.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ComputeGoalLeavingRowActionServer : public rclcpp::Node
{
    public:
        using ComputeBasedGoalAction = field_robot::action::ComputeBasedGoalAction;
        using GoalHandleComputeBasedGoalAction = rclcpp_action::ServerGoalHandle<ComputeBasedGoalAction>;

        ComputeGoalLeavingRowActionServer() : Node("compute_based_goal_action_server")
        {

            this->declare_parameter("action_topic", "front_empty");
            this->get_parameter("action_topic", action_topic);
            this->declare_parameter("robot_frame", "base_footprint");
            this->get_parameter("robot_frame", robot_frame);
            this->declare_parameter("row_width", 0.7);
            this->get_parameter("row_width", row_width);
            
            action_server_ = rclcpp_action::create_server<ComputeBasedGoalAction>(
                this,
                action_topic,
                std::bind(&ComputeGoalLeavingRowActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ComputeGoalLeavingRowActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&ComputeGoalLeavingRowActionServer::handle_accepted, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp_action::Server<ComputeBasedGoalAction>::SharedPtr action_server_;

        std::string action_topic;
        std::string robot_frame;
        double row_width;
        
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ComputeBasedGoalAction::Goal> goal)
        {
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleComputeBasedGoalAction> goal_handle)
        {
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleComputeBasedGoalAction> goal_handle)
        {
            std::thread{std::bind(&ComputeGoalLeavingRowActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleComputeBasedGoalAction> goal_handle)
        {

            const auto goal = goal_handle->get_goal();
            auto result = std::make_shared<ComputeBasedGoalAction::Result>();

            if (goal->turn_direction == true)
            {
                result->pose.pose.position.y = goal->remaining_rows * row_width * -1;
            }else{
                result->pose.pose.position.y = goal->remaining_rows * row_width;
            }

            result->pose.header.frame_id = robot_frame;
            result->pose.header.stamp = goal->pose.header.stamp;
            result->pose.pose.position.x = row_width;
            result->pose.pose.position.z = 0.0;
            tf2::Quaternion tf2_quat;
            tf2_quat.setRPY(0.0, 0.0, 180.0);
            result->pose.pose.orientation = tf2::toMsg(tf2_quat);

            goal_handle->succeed(result);
            return;
        }

};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ComputeGoalLeavingRowActionServer>());
    rclcpp::shutdown();
    return 0;
}
