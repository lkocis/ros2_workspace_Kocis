#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/msg/complex.hpp"
class ComplexPublisherNode : public rclcpp::Node
{
public:
ComplexPublisherNode() : Node("complex_pub")
{
    complex_number_publisher = this->create_publisher<my_interfaces::msg::Complex>("complex_num", 10);
    number_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ComplexPublisherNode::publish_callback, this));
    RCLCPP_INFO(this->get_logger(), "Complex number publisher has been started.");
}
private:
    void publish_callback()
    {
        auto msg = my_interfaces::msg::Complex();
        msg.real = (rand() % 10) / 10.0;
        msg.imaginary = (rand() % 10) / 10.0;
        complex_number_publisher->publish(msg);
    }
rclcpp::Publisher<my_interfaces::msg::Complex>::SharedPtr complex_number_publisher;
rclcpp::TimerBase::SharedPtr number_timer;
};
int main(int argc, char **argv)
{
rclcpp::init(argc, argv);
auto node = std::make_shared<ComplexPublisherNode>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
}