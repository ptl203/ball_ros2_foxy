#include "rclcpp/rclcpp.hpp"
#include "ball_interfaces/srv/start_sequence.hpp"

#include <memory>
using std::placeholders::_1;
using std::placeholders::_2;

class Alpha : public rclcpp::Node
{
    public:
        Alpha() : Node("Alpha") 
        {
            service_ = this->create_service<ball_interfaces::srv::StartSequence>("start_trigger", std::bind(&Alpha::start, this, _1, _2));
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Alpha Node Started");
        }

    private:
        void start(const std::shared_ptr<ball_interfaces::srv::StartSequence::Request> request,
            std::shared_ptr<ball_interfaces::srv::StartSequence::Response> response)
        {
            response->take = "Start Signal Received";
            RCLCPP_INFO(rclcpp::get_logger("rclpp"), "Incoming request: %s", request->give.c_str());
            RCLCPP_INFO(rclcpp::get_logger("rclpp"), "Sending back: %s", response->take.c_str());
        }   
        rclcpp::Service<ball_interfaces::srv::StartSequence>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Alpha>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}