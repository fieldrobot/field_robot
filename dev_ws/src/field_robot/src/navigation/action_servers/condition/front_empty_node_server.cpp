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
            this->declare_parameter("left_cloud_topic", "left_cloud");
            this->get_parameter("left_cloud_topic", left_cloud_topic);
            this->declare_parameter("right_cloud_topic", "right_cloud");
            this->get_parameter("right_cloud_topic", right_cloud_topic);

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.best_effort();
            qos.durability_volatile();
            qos.lifespan(rclcpp::Duration(1, 0));

            subscription_left = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                left_cloud_topic,
                qos,
                std::bind(&FrontEmptyActionServer::topic_callback_left, this, std::placeholders::_1)
            );

            subscription_right = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                right_cloud_topic,
                qos,
                std::bind(&FrontEmptyActionServer::topic_callback_right, this, std::placeholders::_1)
            );
            
            action_server_ = rclcpp_action::create_server<ConditionAction>(
                this,
                action_topic,
                std::bind(&FrontEmptyActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&FrontEmptyActionServer::handle_cancel, this, std::placeholders::_1),
                std::bind(&FrontEmptyActionServer::handle_accepted, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp_action::Server<ConditionAction>::SharedPtr action_server_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_left;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_right;

        std::string action_topic;
        std::string right_cloud_topic;
        std::string left_cloud_topic;

        sensor_msgs::msg::PointCloud2 left_cloud_;
        sensor_msgs::msg::PointCloud2 right_cloud_;

        bool lef = false;
        bool rig = false;

        void topic_callback_left(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            left_cloud_ = *msg;
            lef = true;
        }

        void topic_callback_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            right_cloud_ = *msg;
            rig = true;
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

            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_left_ (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_right_ (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_left_filtered_a (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_right_filtered_a (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_left_filtered_b (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_right_filtered_b (new pcl::PointCloud<pcl::PointXYZ>);

            int j = 0;
            
            while(!goal_handle->is_canceling())
            {

                if ((lef==false)||(rig==false))
                {
                    continue;
                }
                lef = false;
                rig = false;
                
                pcl::fromROSMsg(left_cloud_, *point_cloud_left_);
                pcl::fromROSMsg(right_cloud_, *point_cloud_right_);

                pcl::PassThrough<pcl::PointXYZ> pass_left_a;
                pass_left_a.setInputCloud(point_cloud_left_);
                pass_left_a.setFilterFieldName ("x");
                pass_left_a.setFilterLimits (-0.2, 0.2);
                pass_left_a.filter(*point_cloud_left_filtered_a);
                pcl::PassThrough<pcl::PointXYZ> pass_left_b;
                pass_left_b.setInputCloud(point_cloud_left_filtered_a);
                pass_left_b.setFilterFieldName ("y");
                pass_left_b.setFilterLimits (0.01, 1.0);
                pass_left_b.filter(*point_cloud_left_filtered_b);

                pcl::PassThrough<pcl::PointXYZ> pass_right_a;
                pass_right_a.setInputCloud(point_cloud_right_);
                pass_right_a.setFilterFieldName ("x");
                pass_right_a.setFilterLimits (-0.2, 0.2);
                pass_right_a.filter(*point_cloud_right_filtered_a);
                pcl::PassThrough<pcl::PointXYZ> pass_right_b;
                pass_right_b.setInputCloud(point_cloud_right_filtered_a);
                pass_right_b.setFilterFieldName ("y");
                pass_right_b.setFilterLimits (-1.0, -0.01);
                pass_right_b.filter(*point_cloud_right_filtered_b);

                //RCLCPP_INFO(this->get_logger(), std::to_string(point_cloud_right_filtered_b->size()));
                //RCLCPP_INFO(this->get_logger(), std::to_string(point_cloud_left_filtered_b->size()));

                if ((point_cloud_right_filtered_b->size() < 1) && (point_cloud_left_filtered_b->size() < 1))
                {
                    j = j + 1;
                    /*RCLCPP_INFO(this->get_logger(), "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
                    RCLCPP_INFO(this->get_logger(), "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
                    RCLCPP_INFO(this->get_logger(), "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");*/
                }else{
                    j = 0;
                }

                if (j>10) {
                    //RCLCPP_INFO(this->get_logger(), "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
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
