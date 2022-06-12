#include <set>
#include "behaviortree_cpp_v3/control_node.h"

/*constexpr const char* ParallelCheck::THRESHOLD_FAILURE;
constexpr const char* ParallelCheck::THRESHOLD_SUCCESS;*/

class ParallelCheck : public BT::ControlNode
{
    public:
        ParallelCheck(const std::string& name, unsigned success_threshold,
                                  unsigned failure_threshold)
            : BT::ControlNode::ControlNode(name, {} ),
            success_threshold_(success_threshold),
            failure_threshold_(failure_threshold),
            read_parameter_from_ports_(false)
        {
            setRegistrationID("ParallelCheck");
        }

        ParallelCheck(const std::string &name,
                                    const BT::NodeConfiguration& config)
            : BT::ControlNode::ControlNode(name, config),
            success_threshold_(1),
            failure_threshold_(1),
            read_parameter_from_ports_(true)
        {
        }

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<unsigned>("success_threshold", "number of childen which need to succeed to trigger a SUCCESS" ),
                    BT::InputPort<unsigned>("failure_threshold", 1, "number of childen which need to fail to trigger a FAILURE" ) };
        }

        ~ParallelCheck()
        {
        }

        void halt() override
        {
            skip_list_.clear();
            BT::ControlNode::halt();
        }

        void setThresholdM(unsigned int threshold_M)
        {
            success_threshold_ = threshold_M;
        }

        void setThresholdFM(unsigned int threshold_M)
        {
            failure_threshold_ = threshold_M;
        }

        unsigned int thresholdM();
        unsigned int thresholdFM();

    private:
        unsigned int success_threshold_;
        unsigned int failure_threshold_;

        std::set<int> skip_list_;

        bool read_parameter_from_ports_;
        /*static constexpr const char* THRESHOLD_SUCCESS = "success_threshold";
        static constexpr const char* THRESHOLD_FAILURE = "failure_threshold";*/

        BT::NodeStatus tick()
        {
            if(read_parameter_from_ports_)
            {
                if( !getInput("success_threshold", success_threshold_) )
                {
                    throw BT::RuntimeError("Missing parameter [", "success_threshold", "] in ParallelNode");
                }

                if( !getInput("failure_threshold", failure_threshold_) )
                {
                    throw BT::RuntimeError("Missing parameter [", "failure_threshold", "] in ParallelNode");
                }
            }

            size_t success_childred_num = 0;
            size_t failure_childred_num = 0;

            const size_t children_count = children_nodes_.size();

            if( children_count < success_threshold_)
            {
                throw BT::LogicError("Number of children is less than threshold. Can never succeed.");
            }

            if( children_count < failure_threshold_)
            {
                throw BT::LogicError("Number of children is less than threshold. Can never fail.");
            }

            // Routing the tree according to the sequence node's logic:
            for (unsigned int i = 0; i < children_count; i++)
            {
                TreeNode* child_node = children_nodes_[i];

                bool in_skip_list = (skip_list_.count(i) != 0);

                BT::NodeStatus child_status;
                if( in_skip_list )
                {
                    child_status = child_node->status();
                }
                else {
                    child_status = child_node->executeTick();
                }

                switch (child_status)
                {
                    case BT::NodeStatus::SUCCESS:
                    {
                        if( !in_skip_list )
                        {
                            skip_list_.insert(i);
                        }
                        success_childred_num++;

                        if (success_childred_num == success_threshold_)
                        {
                            skip_list_.clear();
                            haltChildren();
                            return BT::NodeStatus::SUCCESS;
                        }
                    } break;

                    case BT::NodeStatus::FAILURE:
                    {
                        /*if( !in_skip_list )
                        {
                            skip_list_.insert(i);
                        }
                        failure_childred_num++;
                        
                        // It fails if it is not possible to succeed anymore or if 
                        // number of failures are equal to failure_threshold_
                        if ((failure_childred_num > children_count - success_threshold_)
                            || (failure_childred_num == failure_threshold_))
                        {
                            skip_list_.clear();
                            haltChildren();
                            return NodeStatus::FAILURE;
                        }*/
                    } break;

                    case BT::NodeStatus::RUNNING:
                    {
                        // do nothing
                    }  break;

                    default:
                    {
                        throw BT::LogicError("A child node must never return IDLE");
                    }
                }
            }

            return BT::NodeStatus::RUNNING;
        }

};


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<ParallelCheck>("ParallelCheck");
}