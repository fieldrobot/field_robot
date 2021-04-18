#ifndef SIMPLE_BT_NODES_H
#define SIMPLE_BT_NODES_H

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "field_robot/action/emptyspacefollower.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace DummyNodes
{

    BT::NodeStatus CheckBattery()
    {
        std::cout << "[ Battery: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    class ApproachObject : public BT::SyncActionNode
    {
        public:
            ApproachObject(const std::string& name) :
                BT::SyncActionNode(name, {})
            {
            }

            // You must override the virtual function tick()
            BT::NodeStatus tick() override
            {
                std::cout << "ApproachObject: " << this->name() << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
    };

    // Example of custom SyncActionNode (synchronous action)
    // with an input port.
    class SaySomething : public BT::SyncActionNode
    {
        public:
            SaySomething(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
            {
            }

            // You must override the virtual function tick()
            BT::NodeStatus tick() override;

            // It is mandatory to define this static method.
            static BT::PortsList providedPorts()
            {
                return{ BT::InputPort<std::string>("message") };
            }
    };

} // end namespace

#endif   // SIMPLE_BT_NODES_H