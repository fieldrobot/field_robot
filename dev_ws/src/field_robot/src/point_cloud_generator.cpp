#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

class PointCloudGenerator : public rclcpp::Node
{
    public:
        PointCloudGenerator() : Node("point_cloud_generator")
        {
            image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("topic", 10, std::bind(&PointCloudGenerator::image_callback, this, _1));
            image_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("topic", 10);
        }
        //void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->encoding.c_str());
            auto message = sensor_msgs::msg::Image();
            image_publisher_->publish(message);
        }
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudGenerator>());
    rclcpp::shutdown();
    return 0;
}