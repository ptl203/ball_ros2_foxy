#include <memory>
#include <cstdlib>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "ball_interfaces/msg/pub_sub.hpp"
#include "ball_interfaces/srv/start_sequence.hpp"
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
            RCLCPP_INFO(this->get_logger(), "Sending Client Request");

            client_ = this->create_client<ball_interfaces::srv::StartSequence>("ab_connect");
            auto request_ = std::make_shared<ball_interfaces::srv::StartSequence::Request>();
            request_->give = "Send";

            while (!client_->wait_for_service(1s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return 0;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending Request");
            auto result = client_->async_send_request(request_);

            if (rclcpp::spin_until_future_complete(this, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success");
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call services");
            }

        }
        rclcpp::Subscription<ball_interfaces::msg::PubSub>::SharedPtr subscription_;
        rclcpp::Client<ball_interfaces::srv::StartSequence>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Alpha>());
    rclcpp::shutdown();
    return 0;
}