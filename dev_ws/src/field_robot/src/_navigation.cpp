#include <functional>
#include <future>
#include <chrono>
#include <cinttypes>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <unistd.h>

#include "field_robot/action/bt_node.hpp"
#include "field_robot/srv/bt_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

class ROSActionNode : public rclcpp::Node
{
    public:
        using BTNode = field_robot::action::BTNode;
        using GoalHandleBTNode = rclcpp_action::ClientGoalHandle<BTNode>;
        std::shared_future<GoalHandleBTNode::SharedPtr> goal_handle_future;
        bool endedTask;
        bool failure = false;

        ROSActionNode(const std::string & node_name) : Node(node_name)
        {
        }

        void set_action(std::string pNodeName)
        {
            this->client_ptr_ = rclcpp_action::create_client<BTNode>(this, pNodeName);
        }

        void send_goal()
        {
            this->endedTask = false;

            if (!this->client_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            }

            auto goal_msg = BTNode::Goal();

            auto send_goal_options = rclcpp_action::Client<BTNode>::SendGoalOptions();
            send_goal_options.goal_response_callback = std::bind(&ROSActionNode::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.result_callback = std::bind(&ROSActionNode::result_callback, this, std::placeholders::_1);
            this->goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }

        void cancel_goal()
        {
            //this->client_ptr_->async_cancel_goal(GoalHandleBTNode);
            auto goal_handle = this->goal_handle_future.get();
            auto cancel_result_future = client_ptr_->async_cancel_goal(goal_handle);
        }
    
    private:
        rclcpp_action::Client<BTNode>::SharedPtr client_ptr_;
        //GoalHandleBTNode::SharedPtr goal_handle;

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
                this->endedTask = true;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                this->endedTask = true;
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                this->endedTask = true;
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                this->endedTask = true;
                return;
            }
        }

};

namespace Nodes
{

    class BTRosActionNode : public BT::AsyncActionNode
    {
        public:
            std::shared_ptr<ROSActionNode> node;

            explicit BTRosActionNode(const std::string& name, const BT::NodeConfiguration& config) : BT::AsyncActionNode(name, config)
            {
                //rclcpp::init(0, {});

                BT::Optional<std::string> msg = getInput<std::string>("action");
                if (!msg)
                {
                    throw BT::RuntimeError("missing required input [message]: ", 
                                        msg.error() );
                }
                this->node = std::make_shared<ROSActionNode>(this->name());
                this->node->set_action(msg.value());
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
                std::cout << "ticking " << this->name() << std::endl;
                setStatus(BT::NodeStatus::RUNNING);
                this->haltRequested = false;
                this->canceled = false;
                //auto node = std::make_shared<ROSActionNode>(this->name());

                //this->node->set_action(msg.value());
                this->node->send_goal();
                rclcpp::spin_until_future_complete(node, this->node->goal_handle_future);
                while (!this->node->endedTask)
                {
                    rclcpp::spin_some(node);
                    if (this->haltRequested && !this->canceled)
                    {
                        //std::cout << "canceling/halting" << std::endl;
                        this->node->cancel_goal();
                        this->canceled = true;
                    }
                    
                }

                //rclcpp::shutdown();

                if (this->canceled)
                {
                    std::cout << this->name() << " returns failure" << std::endl;
                    return BT::NodeStatus::FAILURE;
                }
                

                std::cout << this->name() << " returns success" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }

            void halt() override
            {

                std::cout << "halting" << std::endl;
                this->haltRequested = true;
            }
        
        private:
            bool haltRequested;
            bool canceled;

    };

    class BTRosServiceNode : public BT::SyncActionNode
    {
        public:
            std::shared_ptr<rclcpp::Node> node;
            std::shared_ptr<field_robot::srv::BTNode::Request> request;
            rclcpp::Client<field_robot::srv::BTNode>::SharedPtr client;

            explicit BTRosServiceNode(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
            {
                //rclcpp::init(0, {});
                BT::Optional<std::string> msg = getInput<std::string>("service");
                if (!msg)
                {
                    throw BT::RuntimeError("missing required input [message]: ", 
                                        msg.error() );
                }
                this->node = rclcpp::Node::make_shared(this->name());
                this->client = this->node->create_client<field_robot::srv::BTNode>(msg.value());
                this->client->wait_for_service();
            }

            static BT::PortsList providedPorts()
            {
                return
                {
                    BT::InputPort<std::string>("service")
                };
            }

            BT::NodeStatus tick() override
            {
                std::cout << "ticking " << this->name() << std::endl;
                setStatus(BT::NodeStatus::RUNNING);

                //rclcpp::Client<field_robot::srv::BTNode>::SharedPtr client = this->node->create_client<field_robot::srv::BTNode>(msg.value());

                this->request = std::make_shared<field_robot::srv::BTNode::Request>();

                auto result = this->client->async_send_request(request);     

                if (rclcpp::spin_until_future_complete(this->node, result) == rclcpp::FutureReturnCode::SUCCESS)
                {
                    if (result.get()->confirm)
                    {
                        std::cout << this->name() << " returns failure" << std::endl;
                        return BT::NodeStatus::FAILURE;
                    }
                }
                //rclcpp::shutdown();
                std::cout << this->name() << " returns success" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
    };

    class BTRosSleepNode : public BT::SyncActionNode
    {
        public:
            explicit BTRosSleepNode(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return {
                    BT::InputPort<int>("duration")
                };
            }

            BT::NodeStatus tick() override
            {
                std::cout << "ticking " << this->name() << std::endl;
                BT::Optional<int> msg = getInput<int>("duration");
                if (!msg)
                {
                    throw BT::RuntimeError("missing required input [message]: ", 
                                        msg.error() );
                }
                sleep(msg.value());
                return BT::NodeStatus::SUCCESS;
            }

    };

}

using namespace BT;

static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence>
            <ReactiveFallback>
                <BTRosServiceNode name="bt_service_in_row" service="/service_in_row"/>
                <BTRosActionNode name="bt_action_empty_space_follower" action="/empty_space_follower"/>
            </ReactiveFallback>
            <BTRosActionNode name="bt_headland_turn" action="/headland_turn"/>
        </Sequence>
     </BehaviorTree>

 </root>
 )";

 /*
 
  <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence>
            <ReactiveFallback>
                <BTRosServiceNode name="bt_service_in_row" service="/service_in_row"/>
                <BTRosActionNode name="bt_action_empty_space_follower" action="/empty_space_follower"/>
            </ReactiveFallback>
            <BTRosActionNode name="bt_headland_turn" action="/headland_turn"/>
        </Sequence>
     </BehaviorTree>

 </root>
 
 */

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    BehaviorTreeFactory factory;

    factory.registerNodeType<Nodes::BTRosActionNode>("BTRosActionNode");
    factory.registerNodeType<Nodes::BTRosServiceNode>("BTRosServiceNode");
    factory.registerNodeType<Nodes::BTRosSleepNode>("BTRosSleepNode");

    auto tree = factory.createTreeFromText(xml_text);

    while (true)
    {
        sleep(0.1);
        tree.tickRoot();
    }

    rclcpp::shutdown();

    return 0;
}