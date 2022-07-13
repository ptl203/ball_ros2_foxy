#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ball_interfaces/srv/start_sequence.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

class Bravo : public rclcpp::Node
{
    public:
        Bravo() : Node("bravo")
        {
            server_ = this->create_service<ball_interfaces::srv::StartSequence>("ab_service", std::bind(&Bravo::send_response_message, this, _1, _2));
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Bravo Node Up");
        }

    private:
        void send_response_message(const ball_interfaces::srv::StartSequence::Request::SharedPtr request,
            ball_interfaces::srv::StartSequence::Response::SharedPtr response)
        {
            response->take = "Received";
            RCLCPP_INFO(this->get_logger(), "I Heard: '%s'", request->give.c_str());
            RCLCPP_INFO(this->get_logger(), "Sending: ' '%s'", response->take.c_str());
        }
        rclcpp::Service<ball_interfaces::srv::StartSequence>::SharedPtr server_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bravo>());
    rclcpp::shutdown();
    return 0;
}