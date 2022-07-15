#include "rclcpp/rclcpp.hpp"
#include "ball_interfaces/msg/pub_sub.hpp"

#include <memory>
using std::placeholders::_1;
using std::placeholders::_2;

class Charlie : public rclcpp::Node
{
    public:
        Charlie() : Node("Bravo"), count_(0) 
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Charlie Node Started");
            subscriber_ = this->create_subscription<ball_interfaces::msg::PubSub>("bc_connect", 10, std::bind(&Charlie::bc_callback, this, _1));
            publisher_ = this->create_publisher<ball_interfaces::msg::PubSub>("ca_connect", 10);
        }
    private:
        void bc_callback(const ball_interfaces::msg::PubSub::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->message.c_str());
            auto message = ball_interfaces::msg::PubSub();
            message.message = "Throwing to Alpha Count: " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.message.c_str());
            publisher_->publish(message);
        }
        rclcpp::Subscription<ball_interfaces::msg::PubSub>::SharedPtr subscriber_;
        rclcpp::Publisher<ball_interfaces::msg::PubSub>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Charlie>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}