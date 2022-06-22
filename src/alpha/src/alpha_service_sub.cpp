#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ball_interfaces/msg/pub_sub.hpp"
using std::placeholders::_1;

class Alpha : public rclcpp::Node
{
    public:
        Alpha() : Node("alpha")
        {
            subscription_ = this->create_subscription<ball_interfaces::msg::PubSub>(
            "ad_connect" , 10, std::bind(&Alpha::callback, this, _1));
            RCLCPP_INFO(this->get_logger(), "Started Alpha Node");
        }

    private:
        void callback(const ball_interfaces::msg::PubSub::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->message.c_str());
        }
        rclcpp::Subscription<ball_interfaces::msg::PubSub>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Alpha>());
    rclcpp::shutdown();
    return 0;
}