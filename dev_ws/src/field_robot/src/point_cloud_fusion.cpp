#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/common/distances.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudFusion : public rclcpp::Node
{
    public:
        PointCloudFusion() : Node("point_cloud_fusion")
        {
            // getting parameters
            // point_cloud_src_1 topic
            this->declare_parameter("pc_src_1", "/camera_one/point_cloud");
            this->get_parameter("pc_src_1", pc_src_1_);
            // point_cloud_src_2 topic
            this->declare_parameter("pc_src_2", "/camera_two/point_cloud");
            this->get_parameter("pc_src_2", pc_src_2_);
            // point_cloud_src_3 topic
            this->declare_parameter("pc_src_3", "/camera_three/point_cloud");
            this->get_parameter("pc_src_3", pc_src_3_);
            // point_cloud_src_4 topic
            this->declare_parameter("pc_src_4", "/camera_four/point_cloud");
            this->get_parameter("pc_src_4", pc_src_4_);
            // point_cloud_dst topic
            this->declare_parameter("pc_dst", "/point_cloud");
            this->get_parameter("pc_dst", pc_dst_);

            // setting quos
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.best_effort();
            qos.durability_volatile();
            qos.lifespan(rclcpp::Duration(1, 0));

            // subscriber
            point_cloud_one_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pc_src_1_, qos, std::bind(&PointCloudFusion::point_cloud_one_callback, this , std::placeholders::_1));
            point_cloud_two_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pc_src_2_, qos, std::bind(&PointCloudFusion::point_cloud_two_callback, this , std::placeholders::_1));
            point_cloud_three_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pc_src_3_, qos, std::bind(&PointCloudFusion::point_cloud_three_callback, this , std::placeholders::_1));
            point_cloud_four_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pc_src_4_, qos, std::bind(&PointCloudFusion::point_cloud_four_callback, this , std::placeholders::_1));

            // publisher
            point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pc_dst_, qos);

            // timer
            timer_ = this->create_wall_timer(std::chrono::milliseconds(40), std::bind(&PointCloudFusion::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            sensor_msgs::msg::PointCloud2 fused_point_cloud;

            pcl::PointCloud<pcl::PointXYZ> comb_1, comb_2, fin;
            pcl::concatenate(point_cloud_one_, point_cloud_two_, comb_1);
            pcl::concatenate(point_cloud_three_, point_cloud_four_, comb_2);
            pcl::concatenate(comb_1, comb_2, fin);

            pcl::toROSMsg(fin, fused_point_cloud);
            fused_point_cloud.header.frame_id = frame_;

            point_cloud_publisher_->publish(fused_point_cloud);
        }

        void point_cloud_one_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            pcl::fromROSMsg(*msg, point_cloud_one_);
            frame_ = msg->header.frame_id;
        }

        void point_cloud_two_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            pcl::fromROSMsg(*msg, point_cloud_two_);
        }

        void point_cloud_three_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            pcl::fromROSMsg(*msg, point_cloud_three_);
        }

        void point_cloud_four_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
            pcl::fromROSMsg(*msg, point_cloud_four_);
        }

        // frame
        std:: string frame_ = "base_footprint";
        
        // topics
        std::string pc_src_1_;
        std::string pc_src_2_;
        std::string pc_src_3_;
        std::string pc_src_4_;
        std::string pc_dst_;

        // point clouds
        pcl::PointCloud<pcl::PointXYZ> point_cloud_one_;
        pcl::PointCloud<pcl::PointXYZ> point_cloud_two_;
        pcl::PointCloud<pcl::PointXYZ> point_cloud_three_;
        pcl::PointCloud<pcl::PointXYZ> point_cloud_four_;

        // publishers and subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_one_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_two_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_three_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_four_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

        // timer
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFusion>());
    rclcpp::shutdown();
    return 0;
}