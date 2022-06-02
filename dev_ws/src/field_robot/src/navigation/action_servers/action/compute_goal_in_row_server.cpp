#include <functional>
#include <memory>
#include <thread>

#include <pcl/common/distances.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "field_robot/action/compute_goal_action.hpp"

class FrontEmptyActionServer : public rclcpp::Node
{
    public:
        using ComputeGoalAction = field_robot::action::ComputeGoalAction;
        using GoalHandleComputeGoalAction = rclcpp_action::ServerGoalHandle<ComputeGoalAction>;

        FrontEmptyActionServer() : Node("front_empty_action_server")
        {

            this->declare_parameter("action_topic", "front_empty");
            this->get_parameter("action_topic", action_topic);
            this->declare_parameter("front_cloud_topic", "front_cloud");
            this->get_parameter("front_cloud_topic", front_cloud_topic);
            this->declare_parameter("back_cloud_topic", "back_cloud");
            this->get_parameter("back_cloud_topic", back_cloud_topic);
            this->declare_parameter("robot_frame", "base_footprint");
            this->get_parameter("robot_frame", robot_frame);

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.best_effort();
            qos.durability_volatile();
            qos.lifespan(rclcpp::Duration(1, 0));

            subscription_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                front_cloud_topic,
                qos,
                std::bind(&FrontEmptyActionServer::topic_front_callback, this, std::placeholders::_1)
            );

            subscription_back_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                back_cloud_topic,
                qos,
                std::bind(&FrontEmptyActionServer::topic_back_callback, this, std::placeholders::_1)
            );
            
            action_server_ = rclcpp_action::create_server<ComputeGoalAction>(
                this,
                action_topic,
                std::bind(&FrontEmptyActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&FrontEmptyActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&FrontEmptyActionServer::handle_accepted, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp_action::Server<ComputeGoalAction>::SharedPtr action_server_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_front_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_back_;


        std::string action_topic;
        std::string front_cloud_topic;
        std::string back_cloud_topic;
        std::string robot_frame;

        sensor_msgs::msg::PointCloud2 cloud_front_;
        sensor_msgs::msg::PointCloud2 cloud_back_;

        void topic_front_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            cloud_front_ = *msg;
        }

        void topic_back_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            cloud_back_ = *msg;
        }
        
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ComputeGoalAction::Goal> goal)
        {
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleComputeGoalAction> goal_handle)
        {
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleComputeGoalAction> goal_handle)
        {
            std::thread{std::bind(&FrontEmptyActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleComputeGoalAction> goal_handle)
        {
            auto result = std::make_shared<ComputeGoalAction::Result>();

            //hier
            pcl::PointCloud<pcl::PointXYZ> point_cloud_front_;
            pcl::PointCloud<pcl::PointXYZ> point_cloud_back_;
            pcl::fromROSMsg(cloud_front_, point_cloud_front_);
            pcl::fromROSMsg(cloud_back_, point_cloud_back_);

            Eigen::Vector4f controid_front;
            Eigen::Vector4f controid_back;

            pcl::compute3DCentroid(point_cloud_front_, controid_front);
            pcl::compute3DCentroid(point_cloud_back_, controid_back);

            result->pose.pose.position.x = (controid_front[0] + (-1 * controid_back[0]))/2;
            result->pose.pose.position.y = (controid_front[1] + (-1 * controid_back[1]))/2;
            result->pose.pose.position.z = (controid_front[2] + (-1 * controid_back[2]))/2;
            result->pose.pose.orientation.x = 0;
            result->pose.pose.orientation.y = 0;
            result->pose.pose.orientation.z = 0;
            result->pose.pose.orientation.w = 1;
            result->pose.header.frame_id = robot_frame;
            result->pose.header.stamp = this->get_clock()->now();

            goal_handle->succeed(result);
            return;

            while(!goal_handle->is_canceling()) {}
        }

};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontEmptyActionServer>());
    rclcpp::shutdown();
    return 0;
}
