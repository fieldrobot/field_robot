#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>

#include "field_robot/action/bt_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class ROSNode : public rclcpp::Node
{
    public:
        using BTNode = field_robot::action::BTNode;
        using GoalHandleBTNode = rclcpp_action::ClientGoalHandle<BTNode>;
        bool endedTask;
        bool failure = false;

        ROSNode() : Node("behavior_tree_node")
        {
        }

        void set_action(std::string pNodeName)
        {
            std::cout << pNodeName << std::endl;
            this->nodeName = pNodeName;
            this->client_ptr_ = rclcpp_action::create_client<BTNode>(this, this->nodeName);
        }

        void send_goal()
        {
            using namespace std::placeholders;

            this->endedTask = false;

            if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            }

            auto goal_msg = BTNode::Goal();

            auto send_goal_options = rclcpp_action::Client<BTNode>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&ROSNode::goal_response_callback, this, _1);
            send_goal_options.result_callback = std::bind(&ROSNode::result_callback, this, _1);
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

        void cancel_goal()
        {
            //this->client_ptr_->async_cancel_goal(GoalHandleBTNode);
        }
    
    private:
        rclcpp_action::Client<BTNode>::SharedPtr client_ptr_;
        std::string nodeName;

        void goal_response_callback(std::shared_future<GoalHandleBTNode::SharedPtr> future)
        {
            auto goal_handle = future.get();
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

        void result_callback(const GoalHandleBTNode::WrappedResult & result)
        {
            switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
            }
            this->endedTask = true;
        }

        /*void cancel_callback(rclcpp::CancelResponse::SharedPtr response)
        {
        }*/

};

namespace Nodes
{

    class EmptySpaceFollowerClientNode : public BT::AsyncActionNode
    {
        public:
            explicit EmptySpaceFollowerClientNode(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config)
            {
                //auto node = std::make_shared<ROSNode>();
            }

            static BT::PortsList providedPorts()
            {
                return
                {
                    BT::InputPort<std::string>("action")
                };
            }

            BT::NodeStatus tick() override
            {
                BT::Optional<std::string> msg = getInput<std::string>("action");
                if (!msg)
                {
                    throw BT::RuntimeError("missing required input [message]: ", 
                                        msg.error() );
                }

                this->haltRequested = false;

                auto node = std::make_shared<ROSNode>();
                node->set_action(msg.value());
                node->send_goal();
                while (!node->endedTask)
                {
                    rclcpp::spin_some(node);
                    if (this->haltRequested)
                    {
                        node->cancel_goal();
                        return BT::NodeStatus::SUCCESS;
                    }
                    
                }

                return BT::NodeStatus::SUCCESS;
            }

            void halt() override
            {
                this->haltRequested = true;
            }
        
        private:
            bool haltRequested;

    };

}

/*namespace DummyNodes
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

} // end namespace*/

/*using namespace BT;

static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <CheckBattery   name="battery_ok"/>
            <ApproachObject name="krank"/>
        </Sequence>
     </BehaviorTree>

 </root>
 )";

int main()
{
    BehaviorTreeFactory factory;

    using namespace DummyNodes;

    factory.registerNodeType<ApproachObject>("ApproachObject");

    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    auto tree = factory.createTreeFromText(xml_text);

    tree.tickRoot();

    return 0;
}*/

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*auto node = std::make_shared<ROSNode>();
    node->set_action("nodine");
    node->send_goal();
    rclcpp::spin(node);*/
    rclcpp::shutdown();
    return 0;
}