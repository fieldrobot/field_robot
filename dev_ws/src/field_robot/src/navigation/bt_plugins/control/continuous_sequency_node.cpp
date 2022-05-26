#include "behaviortree_cpp_v3/control_node.h"

class ContinuousSequence : public BT::ControlNode
{
    public:
        ContinuousSequence(const std::string& name) : BT::ControlNode::ControlNode(name, {}), current_child_idx_(0)
        {
            setRegistrationID("ContinuousSequence");
        }

        void halt()
        {
            current_child_idx_ = 0;
            ControlNode::halt();
        }

    private:
        size_t current_child_idx_;

        BT::NodeStatus tick() override
        {
            const size_t children_count = children_nodes_.size();

            setStatus(BT::NodeStatus::RUNNING);

            while (true)
            {
                BT::TreeNode* current_child_node = children_nodes_[current_child_idx_];
                const BT::NodeStatus child_status = current_child_node->executeTick();

                switch (child_status)
                {
                    case BT::NodeStatus::RUNNING:
                    {
                        return child_status;
                    }
                    case BT::NodeStatus::SUCCESS:
                    {
                        haltChild(current_child_idx_);
                        current_child_idx_++;
                    }
                    break;
                    case BT::NodeStatus::FAILURE:
                    {
                        throw BT::LogicError("A continuous sequence does not expect failure returns.");
                    }
                    case BT::NodeStatus::IDLE:
                    {
                        throw BT::LogicError("A child node must never return IDLE");
                    }
                }

                if(current_child_idx_ >= children_count)
                {
                    current_child_idx_ = 0;
                }
            }

            return BT::NodeStatus::SUCCESS;
        }
};

int main()
{
    return 0;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<ContinuousSequence>("ContinuousSequence");
}