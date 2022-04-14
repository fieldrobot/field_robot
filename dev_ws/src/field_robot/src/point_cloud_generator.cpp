#include <chrono>
#include <functional>
#include <memory>
#include <list>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using std::placeholders::_1;

class PointCloudGenerator : public rclcpp::Node
{
    public:
        PointCloudGenerator() : Node("point_cloud_generator")
        {
            //
            // TODO: get valid frames from camera_info
            //

            // getting parameters
            // image_src topic
            this->declare_parameter("image_src", "/image_ai");
            this->get_parameter("image_src", image_src_);
            // pc_dst topic
            this->declare_parameter("pc_dst", "/point_cloud");
            this->get_parameter("pc_dst", pc_dst_);
            // bord_image topic
            this->declare_parameter("border_img", "/border_image");
            this->get_parameter("border_img", border_img_);

            // setting qos
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.best_effort();
            qos.durability_volatile();
            qos.lifespan(rclcpp::Duration(1, 0));
            // setting up the image subscriber
            image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>((this->get_namespace() + image_src_), qos, std::bind(&PointCloudGenerator::image_callback, this, _1));
            // setting up the point cloud publisher
            pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>((this->get_namespace() + pc_dst_), qos);
            // setting up the border image publisher
            border_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>((this->get_namespace() + border_img_), qos);
            // setting up the transform listener
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
        {
            // converting the image to opencv
            cv_bridge::CvImagePtr cv_bridge_image = cv_bridge::toCvCopy(msg, msg->encoding);
            cv::Mat opencv_image = cv_bridge_image->image;

            // do basic image manipulation
            opencv_image = opencv_image > 180; // making b/w image
            cv::erode(opencv_image, opencv_image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)), cv::Point(-1, -1), 4); // eroding the image
            cv::dilate(opencv_image, opencv_image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)), cv::Point(-1, -1), 16); // eroding the image
            cv::erode(opencv_image, opencv_image, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5)), cv::Point(-1, -1), 4); // eroding the image
            cv::imwrite("/field_robot/img_erode_dilate.jpg", opencv_image);

            /*cv::drawContours(opencv_image, opencv_image, -1, cv::Scalar(0,255,0), -1);
            cv::imwrite("/field_robot/img_contours_filled.jpg", opencv_image); */

            // run edge detection
            cv::Sobel(opencv_image, opencv_image, CV_8U, 0, 1, 3);
            cv::imwrite("/field_robot/img_edge_detection.jpg", opencv_image);
            //cv::Canny(opencv_image, save, 50, 150, 3, false);
            //cv::imwrite("/field_robot/img_canny.jpg", save);

            // publisher border image
            /* cv_bridge_image->image = opencv_image;
            sensor_msgs::msg::Image border_image_msg = *cv_bridge_image->toImageMsg();
            border_image_publisher_->publish(border_image_msg); */

            // identify blobs
            std::list<cv::Point> blob_points;
            RCLCPP_INFO(this->get_logger(), "Starting with %d blobs", blob_points.size());
            cv::Mat nonZeros;
            cv::findNonZero(opencv_image, nonZeros);
            for (int i = 0; i < nonZeros.total(); i++)
            {
                cv::Point point = nonZeros.at<cv::Point>(i);
                blob_points.push_back(point);
            }
            RCLCPP_INFO(this->get_logger(), "Found %d blobs", blob_points.size());
            

            // transform blobs to points (inverted projection)

            // change blob reference frame

            // convert bobs to pc2

            // publish pc2
            
            //geometry_msgs::msg::TransformStamped transformStamped = getTransform();

        }

        geometry_msgs::msg::TransformStamped getTransform() const
        {
            geometry_msgs::msg::TransformStamped transformStamped;

            try {
                transformStamped = tf_buffer_->lookupTransform(camera_frame_, base_frame_, tf2::TimePointZero);
            } catch (tf2::TransformException & ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform.");
                throw;
            }

            return transformStamped;
        }

        std::string camera_frame_ = "camera_link";
        std::string base_frame_ = "base_link";
        std::string image_src_;
        std::string pc_dst_;
        std::string border_img_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr border_image_publisher_;
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