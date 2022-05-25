#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

class NavigationBT : public rclcpp::Node
{
    public:
        NavigationBT() : Node("navigation_bt")
        {
            
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

            BT::SharedLibrary loader;
            for (const auto & p : plugin_lib_names_) {
                factory.registerFromPlugin(loader.getOSName(p));
            }

            tree = factory.createTreeFromFile(xml_file_path_);

            timer_ = this->create_wall_timer(std::chrono::milliseconds(tick_frequency_), std::bind(&NavigationBT::timer_callback, this));
        }
    
    private:
        void timer_callback()
        {
            tree.tickRoot();
        }

        rclcpp::TimerBase::SharedPtr timer_;
        BT::BehaviorTreeFactory factory;
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
