#include "rclcpp/rclcpp.hpp"
#include "ball_interfaces/srv/start_sequence.hpp"

#include <memory>

void kickoff(const std::shared_ptr<ball_interfaces::srv::StartSequence::Request> request,
                std::shared_ptr<ball_interfaces::srv::StartSequence::Response> response)
{
    response->take = "Kickoff!";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received request %s", request.get()->give.c_str());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: %s", response.get()->take.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("alpha");
    rclcpp::Service<ball_interfaces::srv::StartSequence>::SharedPtr service = node->create_service<ball_interfaces::srv::StartSequence>("start_sequence", &kickoff);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to Go.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}