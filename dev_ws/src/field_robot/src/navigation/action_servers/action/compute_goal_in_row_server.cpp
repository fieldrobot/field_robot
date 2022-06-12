#include <functional>
#include <memory>
#include <thread>

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

class ComputeGoalInRowActionServer : public rclcpp::Node
{
    public:
        using ComputeGoalAction = field_robot::action::ComputeGoalAction;
        using GoalHandleComputeGoalAction = rclcpp_action::ServerGoalHandle<ComputeGoalAction>;

        ComputeGoalInRowActionServer() : Node("compute_goal_in_row_action_server")
        {

            RCLCPP_INFO(this->get_logger(), "running constructor of compute goal in row action server");

            this->declare_parameter("action_topic", "front_empty");
            this->get_parameter("action_topic", action_topic);
            /*this->declare_parameter("front_cloud_topic", "front_cloud");
            this->get_parameter("front_cloud_topic", front_cloud_topic);
            this->declare_parameter("back_cloud_topic", "back_cloud");
            this->get_parameter("back_cloud_topic", back_cloud_topic);*/
            this->declare_parameter("robot_frame", "base_footprint");
            this->get_parameter("robot_frame", robot_frame);

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.best_effort();
            qos.durability_volatile();
            //qos.lifespan(rclcpp::Duration(1, 0));

            /*subscription_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                front_cloud_topic,
                qos,
                std::bind(&ComputeGoalInRowActionServer::topic_front_callback, this, std::placeholders::_1)
            );

            subscription_back_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                back_cloud_topic,
                qos,
                std::bind(&ComputeGoalInRowActionServer::topic_back_callback, this, std::placeholders::_1)
            );*/
            
            publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/goal_pose",
                qos
            );

            action_server_ = rclcpp_action::create_server<ComputeGoalAction>(
                this,
                action_topic,
                std::bind(&ComputeGoalInRowActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ComputeGoalInRowActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&ComputeGoalInRowActionServer::handle_accepted, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp_action::Server<ComputeGoalAction>::SharedPtr action_server_;
        //rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_front_;
        //rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_back_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;


        std::string action_topic;
        //std::string front_cloud_topic;
        //std::string back_cloud_topic;
        std::string robot_frame;

        /*sensor_msgs::msg::PointCloud2 cloud_front_;
        sensor_msgs::msg::PointCloud2 cloud_back_;*/

        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        /*void topic_front_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            cloud_front_ = *msg;
        }

        void topic_back_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            cloud_back_ = *msg;
        }*/
        
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
            std::thread{std::bind(&ComputeGoalInRowActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleComputeGoalAction> goal_handle)
        {
            //RCLCPP_INFO(this->get_logger(), "compute goal in row action server is executing");
            
            auto result = std::make_shared<ComputeGoalAction::Result>();

            //hier
            /*pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_front_ (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_front_filtered_ (new pcl::PointCloud<pcl::PointXYZ>);
            //pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_back_, point_cloud_back_filtered_;
            pcl::fromROSMsg(cloud_front_, *point_cloud_front_);
            //pcl::fromROSMsg(cloud_back_, *point_cloud_back_);

            //filtering front
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(point_cloud_front_);
            pass.setFilterFieldName ("y");
            pass.setFilterLimits (-0.5, 0.5);
            pass.filter (*point_cloud_front_filtered_);

            Eigen::Vector4f controid_front;
            //Eigen::Vector4f controid_back;

            pcl::compute3DCentroid(*point_cloud_front_filtered_, controid_front);
            //pcl::compute3DCentroid(*point_cloud_back_filtered_, controid_back);

            RCLCPP_INFO(this->get_logger(), "compute goal in row action server: centroids computed");*/

            geometry_msgs::msg::PoseStamped po;// = geometry_msgs::msg::PoseStamped();
            po.pose.position.x = 1.0;//controid_front[0];//(controid_front[0] + (-1 * controid_back[0]))/2;
            po.pose.position.y = 0.0;//controid_front[1];//(controid_front[1] + (-1 * controid_back[1]))/2;
            po.pose.position.z = 0.0;//(controid_front[2] + (-1 * controid_back[2]))/2;
            po.pose.orientation.x = 0;
            po.pose.orientation.y = 0;
            po.pose.orientation.z = 0;
            po.pose.orientation.w = 1;
            po.header.frame_id = robot_frame;
            po.header.stamp = this->get_clock()->now();

            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("odom", "base_footprint", rclcpp::Time(0, 0), rclcpp::Duration(1, 0));
            tf2::doTransform(po, po, transform);

            result->pose = po;
            publisher_->publish(po);

            //RCLCPP_INFO(this->get_logger(), "compute goal in row action server: stored in result");

            goal_handle->succeed(result);
            //RCLCPP_INFO(this->get_logger(), "succeeded");
            return;

            while(!goal_handle->is_canceling()) {}
        }

};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ComputeGoalInRowActionServer>());
    rclcpp::shutdown();
    return 0;
}
