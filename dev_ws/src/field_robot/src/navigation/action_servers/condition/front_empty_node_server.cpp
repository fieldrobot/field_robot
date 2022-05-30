#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "field_robot/action/condition_action.hpp"

class FrontEmptyActionServer : public rclcpp::Node
{
    public:
        using ConditionAction = field_robot::action::ConditionAction;
        using GoalHandleConditionAction = rclcpp_action::ServerGoalHandle<ConditionAction>;

        FrontEmptyActionServer() : Node("front_empty_action_server")
        {

            this->declare_parameter("action_topic", "front_empty");
            this->get_parameter("action_topic", action_topic);
            this->declare_parameter("front_cloud_topic", "front_cloud");
            this->get_parameter("front_cloud_topic", front_cloud_topic);

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.best_effort();
            qos.durability_volatile();
            qos.lifespan(rclcpp::Duration(1, 0));

            subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                action_topic,
                qos,
                std::bind(&FrontEmptyActionServer::topic_callback, this, std::placeholders::_1)
            );
            
            action_server_ = rclcpp_action::create_server<ConditionAction>(
                this,
                front_cloud_topic,
                std::bind(&FrontEmptyActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&FrontEmptyActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&FrontEmptyActionServer::handle_accepted, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp_action::Server<ConditionAction>::SharedPtr action_server_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

        std::string action_topic;
        std::string front_cloud_topic;

        sensor_msgs::msg::PointCloud2 cloud_;

        void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            cloud_ = *msg;
        }
        
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
            std::thread{std::bind(&FrontEmptyActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleConditionAction> goal_handle)
        {
            auto result = std::make_shared<ConditionAction::Result>();
            while(!goal_handle->is_canceling())
            {
                if (cloud_.data.size() == 0)
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
    rclcpp::spin(std::make_shared<FrontEmptyActionServer>());
    rclcpp::shutdown();
    return 0;
}
