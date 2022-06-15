#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ball_interfaces/msg/pub_sub.hpp"
using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
    public:
        Subscriber() : Node("charlie_subscriber_aclient")
        {
            subscription_ = this->create_subscription<ball_interfaces::msg::PubSub>(
                "pubsub_topic", 10, std::bind(&Subscriber::topic_callback, this, _1));
        }

    private:
        void topic_callback(const ball_interfaces::msg::PubSub::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s' '%d'", msg->prefix.c_str(), msg->counter);
        }
        rclcpp::Subscription<ball_interfaces::msg::PubSub>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Subscriber>());
    rclcpp::shutdown();
    return 0;
}