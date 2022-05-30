#include <functional>
#include <memory>
#include <thread>
#include <iostream>
#include <list>

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

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "field_robot/action/robot_pose_update_action.hpp"

class NewGoalComputableActionServer : public rclcpp::Node
{
    public:
        using RobotPoseUpdateAction = field_robot::action::RobotPoseUpdateAction;
        using GoalHandleRobotPoseUpdateAction = rclcpp_action::ServerGoalHandle<RobotPoseUpdateAction>;

        NewGoalComputableActionServer() : Node("new_goal_computable_action_server")
        {

            this->declare_parameter("action_topic", "new_goal_computable");
            this->get_parameter("action_topic", action_topic);
            this->declare_parameter("left_cloud_topic", "left_cloud");
            this->get_parameter("left_cloud_topic", left_cloud_topic);
            this->declare_parameter("right_cloud_topic", "right_cloud");
            this->get_parameter("right_cloud_topic", right_cloud_topic);
            this->declare_parameter("reference_frame", "odom");
            this->get_parameter("reference_frame", reference_frame);

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.best_effort();
            qos.durability_volatile();
            qos.lifespan(rclcpp::Duration(1, 0));

            subscription_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                right_cloud_topic,
                qos,
                std::bind(&NewGoalComputableActionServer::topic_callback_right, this, std::placeholders::_1)
            );

            subscription_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                left_cloud_topic,
                qos,
                std::bind(&NewGoalComputableActionServer::topic_callback_left, this, std::placeholders::_1)
            );
            
            action_server_ = rclcpp_action::create_server<RobotPoseUpdateAction>(
                this,
                action_topic,
                std::bind(&NewGoalComputableActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&NewGoalComputableActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&NewGoalComputableActionServer::handle_accepted, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp_action::Server<RobotPoseUpdateAction>::SharedPtr action_server_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_left_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_right_;

        std::string action_topic;
        std::string left_cloud_topic;
        std::string right_cloud_topic;
        std::string reference_frame;

        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right_;

        void topic_callback_left(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            pcl::fromROSMsg(*msg, *cloud_left_);
        }

        void topic_callback_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            pcl::fromROSMsg(*msg, *cloud_right_);
        }
        
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RobotPoseUpdateAction::Goal> goal)
        {
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleRobotPoseUpdateAction> goal_handle)
        {
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandleRobotPoseUpdateAction> goal_handle)
        {
            std::thread{std::bind(&NewGoalComputableActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandleRobotPoseUpdateAction> goal_handle)
        {
            const auto goal = goal_handle->get_goal();
            auto result = std::make_shared<RobotPoseUpdateAction::Result>();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, left, right;

            if (goal->turn_direction)
            {
                cloud = cloud_right_;
            } else {
                cloud = cloud_left_;
            }
            float f;

            pcl::PassThrough<pcl::PointXYZ> pass_left, pass_right;
            pass_right.setInputCloud(cloud);
            pass_right.setFilterFieldName("x");
            pass_right.setFilterLimits(0.0, 0.7);
            pass_left.setInputCloud(cloud);
            pass_left.setFilterFieldName("x");
            pass_left.setFilterLimits(-0.7, 0.0);

            do{
                pass_right.filter(*right);
                pass_left.filter(*left);
                f = left->size()/right->size();
            }while(!((0.625<f)&&(f<1.6)));

            std::list<geometry_msgs::msg::PointStamped> list = {};
            geometry_msgs::msg::PointStamped point;
            
            do{
                pass_right.filter(*right);
                pass_left.filter(*left);
                f = left->size()/right->size();

                if ((0.75<f)&&(f<1.3))
                {
                    //left
                    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));
                    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_l (model_l);
                    ransac_l.setDistanceThreshold(.05);
                    ransac_l.computeModel();
                    Eigen::VectorXf coefficients_l;
                    ransac_l.getModelCoefficients(coefficients_l);
                    tf_buffer_->transform<geometry_msgs::msg::PointStamped>(get_point(cloud->header.frame_id, coefficients_l(0,0), coefficients_l(1,0), coefficients_l(2,0)), point, reference_frame, tf2::durationFromSec(0.0));
                    list.push_back(point);

                    //right
                    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_r(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));
                    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_r (model_r);
                    ransac_r.setDistanceThreshold(.05);
                    ransac_r.computeModel();
                    Eigen::VectorXf coefficients_r;
                    ransac_r.getModelCoefficients(coefficients_r);
                    tf_buffer_->transform<geometry_msgs::msg::PointStamped>(get_point(cloud->header.frame_id, coefficients_r(0,0), coefficients_r(1,0), coefficients_r(2,0)), point, reference_frame, tf2::durationFromSec(0.0));
                    list.push_back(point);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }while((0.588<f)&&(f<1.7));

            float avg_x = 0;
            float avg_y = 0;
            for (int i = 0; i < list.size(); i++)
            {
                avg_x += list.front().point.x;
                avg_y += list.front().point.y;
                list.pop_front();
            }
            avg_x /= list.size();
            avg_y /= list.size();

            result->pose.pose.position.x = avg_x;
            result->pose.pose.position.y = avg_y;
            result->pose.pose.position.z = 0.0;
            result->pose.header.frame_id = reference_frame;
            
            goal_handle->succeed(result);
            return;
            
                
            while(!goal_handle->is_canceling())
            {}
        }

        geometry_msgs::msg::PointStamped get_point(std::string id, float x, float y, float z) const
        {
            geometry_msgs::msg::PointStamped point;
            point.header.frame_id = id;
            point.point.x = x;
            point.point.y = y;
            point.point.z = z;
            return point;
        }

};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NewGoalComputableActionServer>());
    rclcpp::shutdown();
    return 0;
}