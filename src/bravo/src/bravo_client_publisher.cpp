#include "rclcpp/rclcpp.hpp"
#include "ball_interfaces/srv/start_sequence.hpp"
#include "ball_interfaces/msg/pub_sub.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <functional>
#include <string>

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
    public:
        Publisher() : Node("bravo_publisher")
        {
            publisher_ = this->create_publisher<ball_interfaces::msg::PubSub>("pubsub_topic", 10);
            timer_ = this->create_wall_timer(500ms, std::bind(&Publisher::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            auto message = ball_interfaces::msg::PubSub();
            message.prefix = "Count is: ";
            message.counter = count_++;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s' '%d'", message.prefix.c_str(), message.counter);
            publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ball_interfaces::msg::PubSub>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 2)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: bravo_client_publisher Kickoff?");
        return 1;
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bravo_client");
    rclcpp::Client<ball_interfaces::srv::StartSequence>::SharedPtr client =
        node->create_client<ball_interfaces::srv::StartSequence>("start_sequence");

    auto request = std::make_shared<ball_interfaces::srv::StartSequence::Request>();
    request->give = atoll(argv[1]);

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received Result: %s", result.get()->take.c_str());
        rclcpp::spin(std::make_shared<Publisher>());
    } else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service start_sequence");
    }
    rclcpp::shutdown();
    return 0;
}