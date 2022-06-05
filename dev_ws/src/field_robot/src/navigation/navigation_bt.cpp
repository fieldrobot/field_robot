#include <string>
#include <iostream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include <tf2_ros/buffer.h>

class NavigationBT : public rclcpp::Node
{
    public:
        NavigationBT() : Node("navigation_bt")
        {


            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
            
            RCLCPP_INFO(this->get_logger(), "starting navigation bt node");
            std::cout << "DAS GOAL WURDE GESETZT (2) !!!" << std::endl;

            // getting parameters
            // xml file path
            this->declare_parameter("xml_file_path");
            this->get_parameter("xml_file_path", xml_file_path_);
            // plugin lib names
            this->declare_parameter("plugin_lib_names");
            this->get_parameter("plugin_lib_names", plugin_lib_names_);
            // tick frequency
            this->declare_parameter("tick_frequency_ms");
            this->get_parameter("tick_frequency_ms", tick_frequency_);

            RCLCPP_INFO(this->get_logger(), "loading bt nodes");

            BT::SharedLibrary loader;
            for (const auto & p : plugin_lib_names_) {
                factory.registerFromPlugin(loader.getOSName(p));
            }

            RCLCPP_INFO(this->get_logger(), "loading the behavior tree from xml file");


            std::shared_ptr<tf2_ros::Buffer> tf_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

            /*tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
            auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                get_node_base_interface(), get_node_timers_interface());
            tf_->setCreateTimerInterface(timer_interface);
            tf_->setUsingDedicatedThread(true);
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            try {
                auto hi = tf_buffer_->lookupTransform("odom", "base_footprint", rclcpp::Time(0, 0), tf2::durationFromSec(5.0));
            } catch(const std::exception& e) {}*/

            blackboard_ = BT::Blackboard::create();
            blackboard_->set<rclcpp::Node::SharedPtr>("node", std::make_shared<rclcpp::Node>("bt_client_node"));
            blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(1000));
            //blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_);

            tree = factory.createTreeFromFile(xml_file_path_, blackboard_);

            RCLCPP_INFO(this->get_logger(), "creating the timer for tree ticking");

            timer_ = this->create_wall_timer(std::chrono::milliseconds(tick_frequency_), std::bind(&NavigationBT::timer_callback, this));
        }
    
    private:
        /*std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;*/

        void timer_callback()
        {
            RCLCPP_INFO(this->get_logger(), "calling timer callback, ticking the tree");
            tree.tickRoot();
        }

        rclcpp::TimerBase::SharedPtr timer_;
        BT::BehaviorTreeFactory factory;
        BT::Blackboard::Ptr blackboard_;
        BT::Tree tree;

        std::string xml_file_path_;
        std::vector<std::string> plugin_lib_names_;
        int tick_frequency_;

};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationBT>());
    rclcpp::shutdown();
    return 0;

}
