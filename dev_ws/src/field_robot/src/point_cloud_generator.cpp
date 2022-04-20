#include <chrono>
#include <functional>
#include <memory>
#include <list>

#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/common/distances.h>
#include <pcl/impl/point_types.hpp>
//#include <pcl_conversions/pcl_conversions.h>

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
            // camera_info topic
            this->declare_parameter("camera_info", "/camera_info");
            this->get_parameter("camera_info", camera_info_);

            // setting qos
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.best_effort();
            qos.durability_volatile();
            qos.lifespan(rclcpp::Duration(1, 0));
            // setting up the image subscriber
            image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>((this->get_namespace() + image_src_), qos, std::bind(&PointCloudGenerator::image_callback, this, _1));
            // setting up the camera info subscriber
            camera_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>((this->get_namespace() + camera_info_), qos, std::bind(&PointCloudGenerator::camera_info_callback, this, _1));
            // setting up the point cloud publisher
            pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>((this->get_namespace() + pc_dst_), qos);
            // setting up the border image publisher
            border_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>((this->get_namespace() + border_img_), qos);
            // setting up the transform listener
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            // transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
            

            // transform blobs to points & changing reference frame
            geometry_msgs::msg::Vector3Stamped points_camera_frame[blob_points.size()];
            for (int i = 0; i < blob_points.size(); i++)
            {
                tf_buffer_->transform<geometry_msgs::msg::Vector3Stamped>(pixelCooridnates2Vector(blob_points.front().x, blob_points.front().y), points_camera_frame[i], base_frame_);
                blob_points.pop_front();
            }

            // find groud points
            geometry_msgs::msg::PointStamped ground_points[blob_points.size()];
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(base_frame_, camera_frame_, rclcpp::Time(0, 0), tf2::durationFromSec(0.1));
            float64_t z_diff = transform.transform.translation.z;
            for (int i = 0; i < sizeof(points_camera_frame); i++)
            {
                float64_t factor = z_diff/(points_camera_frame[i].vector.z);
                points_camera_frame[i].vector.x = points_camera_frame[i].vector.x * factor;
                points_camera_frame[i].vector.y = points_camera_frame[i].vector.y * factor;
                points_camera_frame[i].vector.z = points_camera_frame[i].vector.z * factor;
                ground_points[i].point.x = points_camera_frame[i].vector.x - transform.transform.translation.x;
                ground_points[i].point.y = points_camera_frame[i].vector.y - transform.transform.translation.y;
                ground_points[i].point.z = points_camera_frame[i].vector.z - transform.transform.translation.z;
                ground_points[i].header.frame_id = base_frame_;
            }    

            // convert bobs to pc2
            pcl::PointCloud<pcl::PointXYZ> cloud;
            for (int i = 0; i < sizeof(ground_points); i++)
            {
                pcl::PointXYZ point = pcl::PointXYZ(ground_points[i].point.x, ground_points[i].point.y, ground_points[i].point.z);
                /*point.x = ground_points[i].point.x;
                point.y = ground_points[i].point.y;
                point.z = ground_points[i].point.z;*/
                cloud.push_back(point);
            }

            // https://answers.ros.org/question/312587/generate-and-publish-pointcloud2-in-ros2/
            // https://github.com/ros-perception/perception_pcl/tree/foxy-devel/pcl_ros
            // http://docs.ros.org/en/indigo/api/pcl_conversions/html/namespacepcl.html
            // https://pointclouds.org/documentation/classpcl_1_1_point_cloud.html

            // publish pc2
            
            //geometry_msgs::msg::TransformStamped transformStamped = getTransform();

        }

        geometry_msgs::msg::Vector3Stamped pixelCooridnates2Vector(int x, int y) const
        {
            geometry_msgs::msg::Vector3Stamped vectorStamped;
            vectorStamped.vector.x = (x - cx_)/fx_;
            vectorStamped.vector.y = (y - cy_)/fy_;
            vectorStamped.vector.z = 1;
            vectorStamped.header.frame_id = camera_frame_;
            return vectorStamped;
        }
        
        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
        {
            fx_ = msg->p[0];
            cx_ = msg->p[2];
            fy_ = msg->p[5];
            cy_ = msg->p[6];
        }

        /*geometry_msgs::msg::TransformStamped getTransform() const
        {
            geometry_msgs::msg::TransformStamped transformStamped;

            try {
                transformStamped = tf_buffer_->lookupTransform(camera_frame_, base_frame_, tf2::TimePointZero);
            } catch (tf2::TransformException & ex) {
                RCLCPP_INFO(this->get_logger(), "Could not transform.");
                throw;
            }

            return transformStamped;
        }*/

        // frames
        std::string camera_frame_ = "camera_link";
        std::string base_frame_ = "base_link";

        //topics
        std::string image_src_;
        std::string pc_dst_;
        std::string border_img_;
        std::string camera_info_;

        // global variables
        float64_t fx_ = 183.5583030605572;
        float64_t fy_ = 183.5583030605572;
        float64_t cx_ = 320;
        float64_t cy_ = 240;

        // publishers and subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscriber_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr border_image_publisher_;

        // transform listener
        // std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudGenerator>());
    rclcpp::shutdown();
    return 0;
}