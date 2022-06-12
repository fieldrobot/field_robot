#include <functional>
#include <memory>
#include <thread>
#include <chrono>

#include <pcl/common/distances.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "field_robot/action/compute_goal_action.hpp"

class ComputeGoalInHeadlandActionServer : public rclcpp::Node
{
    public:
        using ComputeGoalAction = field_robot::action::ComputeGoalAction;
        using GoalHandleComputeGoalAction = rclcpp_action::ServerGoalHandle<ComputeGoalAction>;

        ComputeGoalInHeadlandActionServer() : Node("compute_goal_in_headland_action_server")
        {

            RCLCPP_INFO(this->get_logger(), "running constructor of compute goal in headland action server");

            this->declare_parameter("action_topic", "front_empty");
            this->get_parameter("action_topic", action_topic);
            this->declare_parameter("action_topic_follow", "front_empty_follow");
            this->get_parameter("action_topic_follow", action_topic_follow);
            this->declare_parameter("robot_frame", "base_footprint");
            this->get_parameter("robot_frame", robot_frame);

            //auto kevin = std::chrono::nanoseconds(500000000);
            //auto adf = kevin * 5;
            uint64_t wte = 8*uint64_t(1000000000);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), std::chrono::nanoseconds(wte));
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.best_effort();
            qos.durability_volatile();
            
            publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/goal_pose",
                qos
            );

            action_server_ = rclcpp_action::create_server<ComputeGoalAction>(
                this,
                action_topic,
                std::bind(&ComputeGoalInHeadlandActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ComputeGoalInHeadlandActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&ComputeGoalInHeadlandActionServer::handle_accepted, this, std::placeholders::_1)
            );

            action_server_follow_ = rclcpp_action::create_server<ComputeGoalAction>(
                this,
                action_topic_follow,
                std::bind(&ComputeGoalInHeadlandActionServer::handle_goal_follow, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ComputeGoalInHeadlandActionServer::handle_cancel_follow, this, std::placeholders::_1),
                std::bind(&ComputeGoalInHeadlandActionServer::handle_accepted_follow, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp_action::Server<ComputeGoalAction>::SharedPtr action_server_;
        rclcpp_action::Server<ComputeGoalAction>::SharedPtr action_server_follow_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;

        geometry_msgs::msg::PoseStamped poa;

        std::string action_topic;
        std::string action_topic_follow;
        std::string robot_frame;

        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ComputeGoalAction::Goal> goal)
        {
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::GoalResponse handle_goal_follow(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ComputeGoalAction::Goal> goal)
        {
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleComputeGoalAction> goal_handle)
        {
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        rclcpp_action::CancelResponse handle_cancel_follow(const std::shared_ptr<GoalHandleComputeGoalAction> goal_handle)
        {
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleComputeGoalAction> goal_handle)
        {
            std::thread{std::bind(&ComputeGoalInHeadlandActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void handle_accepted_follow(const std::shared_ptr<GoalHandleComputeGoalAction> goal_handle)
        {
            std::thread{std::bind(&ComputeGoalInHeadlandActionServer::execute_follow, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleComputeGoalAction> goal_handle)
        {
            //RCLCPP_INFO(this->get_logger(), "compute goal in headland action server is executing");
            
            auto result = std::make_shared<ComputeGoalAction::Result>();

            geometry_msgs::msg::PoseStamped po;// = geometry_msgs::msg::PoseStamped();
            poa.pose.position.x = 0.0;
            poa.pose.position.y = 2*1.2;
            poa.pose.position.z = 0.0;
            poa.pose.orientation.x = 0;
            poa.pose.orientation.y = 0;
            poa.pose.orientation.z = 1;
            poa.pose.orientation.w = 0;
            poa.header.frame_id = robot_frame;
            poa.header.stamp = this->get_clock()->now();

            po.pose.position.x = 1.5;
            po.pose.position.y = 2*1.2;
            po.pose.position.z = 0.0;
            po.pose.orientation.x = 0;
            po.pose.orientation.y = 0;
            po.pose.orientation.z = 1;//0.707;
            po.pose.orientation.w = 0;//0.707;
            po.header.frame_id = robot_frame;
            po.header.stamp = this->get_clock()->now();

            auto ni = this->get_clock()->now();
            rclcpp::Time no = ni-rclcpp::Duration(7, 0);

            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("odom", "base_footprint", no, rclcpp::Duration(1, 0));
            tf2::doTransform(po, po, transform);
            tf2::doTransform(poa, poa, transform);

            result->pose = po;
            publisher_->publish(po);

            //RCLCPP_INFO(this->get_logger(), "compute goal in headland action server: stored in result");

            goal_handle->succeed(result);
            //RCLCPP_INFO(this->get_logger(), "succeeded");
            return;

            while(!goal_handle->is_canceling()) {}
        }

        void execute_follow(const std::shared_ptr<GoalHandleComputeGoalAction> goal_handle)
        {
            auto result = std::make_shared<ComputeGoalAction::Result>();
            result->pose = poa;
            publisher_->publish(poa);
            goal_handle->succeed(result);
            return;
        }

};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ComputeGoalInHeadlandActionServer>());
    rclcpp::shutdown();
    return 0;
}