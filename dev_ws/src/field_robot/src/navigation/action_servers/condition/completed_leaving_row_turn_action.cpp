#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <tf2/exceptions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "geometry_msgs/msg/pose.hpp"
#include "field_robot/action/condition_action.hpp"

class CompletedLeavingRowActionServer : public rclcpp::Node
{
    public:
        using ConditionAction = field_robot::action::ConditionAction;
        using GoalHandleConditionAction = rclcpp_action::ServerGoalHandle<ConditionAction>;

        CompletedLeavingRowActionServer() : Node("completed_leaving_row_action_server")
        {

            this->declare_parameter("action_topic", "front_empty");
            this->get_parameter("action_topic", action_topic);
            this->declare_parameter("base_frame", "odom");
            this->get_parameter("base_frame", base_frame);
            this->declare_parameter("robot_frame", "base_footprint");
            this->get_parameter("robot_frame", robot_frame);

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            
            action_server_ = rclcpp_action::create_server<ConditionAction>(
                this,
                action_topic,
                std::bind(&CompletedLeavingRowActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&CompletedLeavingRowActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&CompletedLeavingRowActionServer::handle_accepted, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp_action::Server<ConditionAction>::SharedPtr action_server_;

        std::string action_topic;
        std::string base_frame;
        std::string robot_frame;

        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ConditionAction::Goal> goal)
        {
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleConditionAction> goal_handle)
        {
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleConditionAction> goal_handle)
        {
            std::thread{std::bind(&CompletedLeavingRowActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleConditionAction> goal_handle)
        {
            auto result = std::make_shared<ConditionAction::Result>();

            geometry_msgs::msg::TransformStamped transform;
            try
            {
                transform = tf_buffer_->lookupTransform(base_frame, robot_frame, rclcpp::Time(0, 0), tf2::durationFromSec(0.0));
            }
            catch(const std::exception& e)
            {
                RCLCPP_INFO(this->get_logger(), "Could not transform !!!");
                return;
            }
            tf2::Quaternion original_quaternion;// = transform.transform.rotation;
            tf2::convert(transform.transform.rotation, original_quaternion);

            while(!goal_handle->is_canceling())
            {

                tf2::Quaternion current, relative;
                tf2::convert((tf_buffer_->lookupTransform(base_frame, robot_frame, rclcpp::Time(0, 0), tf2::durationFromSec(0.0))).transform.rotation, current);
                relative = original_quaternion * current.inverse();

                if (relative.getAngle() <= 1,0472)
                {
                    result->success = true;
                    goal_handle->succeed(result);
                    return;
                }
                
            }
        }

};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CompletedLeavingRowActionServer>());
    rclcpp::shutdown();
    return 0;
}
