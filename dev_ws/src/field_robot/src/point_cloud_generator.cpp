#include "rclcpp/rclcpp.hpp"
#include "sensor_msg/msg/Image.hpp"

class PointCloudGenerator : public rclcpp::Node
{
    public:
        PointCloudGenerator() : Node("point_cloud_generator")
        {
            image_usbscriber_ = this->create_subscription<sensor_msg::msg::Image>("topic", 10, std::bind(&PointCloudGenerator::image_callback, this, _1));
            image_publisher_ = this->create_publisher<sensor_msg::msg::Image>("topic", 10);
        }
        void image_callback(const sensor_msg::msg::Image::SharedPtr msg);

    private:
        void image_callback(const sensor_msg::msg::Image::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->header.c_str());
            auto message = sensor_msg::msg::Image();
            iamge_publisher_->publish(message);
        }
        rclcpp::Subscription<sensor_msg::msg::Image>::SharedPtr image_subscriber_;
        rclcpp::Publisher<sensor_msg::msg::Image>::SharedPtr image_publisher_;
}

int main(int, argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudGenerator>());
    rclcpp::shutdown();
    return 0;
}