#include "rclcpp/rclcpp.hpp"
class MyNode : public rclcpp::Node
{
    public:
        MyNode() : Node("cpp_ros2_node")
        {
            m_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode::print_hello, this));
        }
        void print_hello()
        {
            RCLCPP_INFO(this->get_logger(), "Hello Lana %d. time", m_counter);
            m_counter++;
        }
    private:
        int m_counter;
        rclcpp::TimerBase::SharedPtr m_timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}