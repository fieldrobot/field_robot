#include "behaviortree_cpp_v3/decorator_node.h"

class IndefiniteRepeat : public BT::DecoratorNode
{
    public:
        IndefiniteRepeat(const std::string& name) : BT::DecoratorNode(name, {})
        {
            setRegistrationID("IndefiniteRepeat");
        }

        void halt()
        {
            DecoratorNode::halt();
        }

    private:
        BT::NodeStatus tick() override
        {
            setStatus(BT::NodeStatus::RUNNING);
            child_node_->executeTick();

            return BT::NodeStatus::RUNNING;
        }
};

int main()
{
    return 0;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<IndefiniteRepeat>("IndefiniteRepeat");
}