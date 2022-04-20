#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

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
            point_cloud_one_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>((this->get_namespace() + pc_src_1_), qos, std::bind(&PointCloudFusion::point_cloud_one_callback, this , std::placeholders::_1));
            point_cloud_two_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>((this->get_namespace() + pc_src_2_), qos, std::bind(&PointCloudFusion::point_cloud_two_callback, this , std::placeholders::_1));
            point_cloud_three_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>((this->get_namespace() + pc_src_3_), qos, std::bind(&PointCloudFusion::point_cloud_three_callback, this , std::placeholders::_1));
            point_cloud_four_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>((this->get_namespace() + pc_src_4_), qos, std::bind(&PointCloudFusion::point_cloud_four_callback, this , std::placeholders::_1));

            // publisher
            point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>((this->get_namespace() + pc_dst_), qos);

            // timer
            timer_ = this->create_wall_timer(std::chrono::milliseconds(40), std::bind(&PointCloudFusion::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            sensor_msgs::msg::PointCloud2 fused_point_cloud;
        }

        void point_cloud_one_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
        {

        }

        void point_cloud_two_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
        {

        }

        void point_cloud_three_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
        {

        }

        void point_cloud_four_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
        {

        }

        std::string pc_src_1_;
        std::string pc_src_2_;
        std::string pc_src_3_;
        std::string pc_src_4_;
        std::string pc_dst_;

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