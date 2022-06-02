#include <functional>
#include <memory>
#include <thread>
#include <math.h>

#include <pcl/common/distances.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "field_robot/action/compute_based_goal_action.hpp"

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ComputeGoalHeadlandDriveActionServer : public rclcpp::Node
{
    public:
        using ComputeBasedGoalAction = field_robot::action::ComputeBasedGoalAction;
        using GoalHandleComputeBasedGoalAction = rclcpp_action::ServerGoalHandle<ComputeBasedGoalAction>;

        ComputeGoalHeadlandDriveActionServer() : Node("compute_based_goal_headland_drive_action_server")
        {

            this->declare_parameter("action_topic", "front_empty");
            this->get_parameter("action_topic", action_topic);
            this->declare_parameter("front_cloud_topic", "front_cloud");
            this->get_parameter("front_cloud_topic", front_cloud_topic);
            this->declare_parameter("back_cloud_topic", "back_cloud");
            this->get_parameter("back_cloud_topic", back_cloud_topic);
            this->declare_parameter("robot_frame", "base_footprint");
            this->get_parameter("robot_frame", robot_frame);
            this->declare_parameter("row_width", 0.7);
            this->get_parameter("row_width", row_width);

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.best_effort();
            qos.durability_volatile();
            qos.lifespan(rclcpp::Duration(1, 0));

            subscription_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                front_cloud_topic,
                qos,
                std::bind(&ComputeGoalHeadlandDriveActionServer::topic_front_callback, this, std::placeholders::_1)
            );

            subscription_back_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                back_cloud_topic,
                qos,
                std::bind(&ComputeGoalHeadlandDriveActionServer::topic_back_callback, this, std::placeholders::_1)
            );
            
            action_server_ = rclcpp_action::create_server<ComputeBasedGoalAction>(
                this,
                action_topic,
                std::bind(&ComputeGoalHeadlandDriveActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ComputeGoalHeadlandDriveActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&ComputeGoalHeadlandDriveActionServer::handle_accepted, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp_action::Server<ComputeBasedGoalAction>::SharedPtr action_server_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_front_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_back_;

        std::string action_topic;
        std::string robot_frame;
        std::string front_cloud_topic;
        std::string back_cloud_topic;
        double row_width;

        sensor_msgs::msg::PointCloud2 cloud_front_;
        sensor_msgs::msg::PointCloud2 cloud_back_;

        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        void topic_front_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            cloud_front_ = *msg;
        }

        void topic_back_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            cloud_back_ = *msg;
        }
        
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
            std::thread{std::bind(&ComputeGoalHeadlandDriveActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleComputeBasedGoalAction> goal_handle)
        {

            const auto goal = goal_handle->get_goal();
            auto result = std::make_shared<ComputeBasedGoalAction::Result>();

            pcl::PointCloud<pcl::PointXYZ>::Ptr comb, fr, bk;
            pcl::fromROSMsg(cloud_front_, *fr);
            pcl::fromROSMsg(cloud_back_, *bk);

            pcl::concatenate(*fr, *bk, *comb);

            pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (comb));
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);
            ransac.setDistanceThreshold(.05);
            ransac.computeModel();
            Eigen::VectorXf line;
            ransac.getModelCoefficients(line);

            geometry_msgs::msg::Point start;
            start.x = goal->pose.pose.position.x;
            start.y = goal->pose.pose.position.y;
            start.z = goal->pose.pose.position.z;

            auto line_point = closest_point_on_line(start, line);
            line[0] = line_point.x;
            line[1] = line_point.y;
            line[2] = line_point.z;

            geometry_msgs::msg::Point vector;
            vector.x = line[0]-start.x;
            vector.y = line[1]-start.y;
            vector.z = line[2]-start.z;

            geometry_msgs::msg::Point vec_start;
            vec_start.x = -line[0];
            vec_start.y = -line[1];
            vec_start.z = -line[2];

            if (get_smallest_angle_between_vectors(vector, vec_start)>90)
            {
                vector.x = -vector.x;
                vector.y = -vector.y;
                vector.z = -vector.z;
            }
            
            if (goal->turn_direction)
            {
                geometry_msgs::msg::Point turned = clockwise(vector);
                line[3] = turned.x;
                line[4] = turned.y;
                line[5] = turned.z;
            }else{
                geometry_msgs::msg::Point turned = counter_clockwise(vector);
                line[3] = turned.x;
                line[4] = turned.y;
                line[5] = turned.z;
            }
            
            double length_of_line_orientation = sqrt(pow(line[3],2)+pow(line[4],2)+pow(line[5],2));
            double k = (goal->remaining_rows * row_width)/(length_of_line_orientation);

            geometry_msgs::msg::Point po;
            po.x = line[0] + k*line[3];
            po.y = line[1] + k*line[4];
            po.z = line[2] + k*line[5];

            double length_of_vector = sqrt(pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2));
            double l = 0.4/length_of_vector;

            result->pose.header.frame_id = cloud_front_.header.frame_id;
            result->pose.header.stamp = cloud_front_.header.stamp;
            result->pose.pose.position.x = po.x + l*vector.x;
            result->pose.pose.position.y = po.y + l*vector.y;
            result->pose.pose.position.z = po.z + l*vector.z;
            result->pose.pose.orientation = goal->pose.pose.orientation;

            goal_handle->succeed(result);
            return;
        }

        geometry_msgs::msg::Point closest_point_on_line(geometry_msgs::msg::Point p, Eigen::VectorXf coefficients)
        {
            geometry_msgs::msg::Point closest;
            return closest;
        }

        double get_smallest_angle_between_vectors(geometry_msgs::msg::Point a, geometry_msgs::msg::Point b)
        {
            return 0.0;
        }

        geometry_msgs::msg::Point clockwise(geometry_msgs::msg::Point vector)
        {
            return vector;
        }

        geometry_msgs::msg::Point counter_clockwise(geometry_msgs::msg::Point vector)
        {
            return vector;
        }

};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ComputeGoalHeadlandDriveActionServer>());
    rclcpp::shutdown();
    return 0;
}
