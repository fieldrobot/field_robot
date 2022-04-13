#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

using std::placeholders::_1;

class PointCloudGenerator : public rclcpp::Node
{
    public:
        PointCloudGenerator() : Node("point_cloud_generator")
        {
            image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("topic", 10, std::bind(&PointCloudGenerator::image_callback, this, _1));
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.best_effort();
            qos.durability_volatile();
            qos.lifespan(rclcpp::Duration(1, 0));
            image_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("topic", qos);

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }
        //void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const;

    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->encoding.c_str());
            auto message = sensor_msgs::msg::Image();
            //image_publisher_->publish(message);

            geometry_msgs::msg::TransformStamped transformStamped;

            try {
                transformStamped = tf_buffer_->lookupTransform("base_link", "base_linnk", tf2::TimePointZero);
            } catch (tf2::TransformException & ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform.");
                return;
            }

        }
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr image_publisher_;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudGenerator>());
    rclcpp::shutdown();
    return 0;
}