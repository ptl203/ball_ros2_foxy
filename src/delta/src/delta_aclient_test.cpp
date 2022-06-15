#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "ball_interfaces/action/ball.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace delta
{
    class BallActionClient : public rclcpp::Node
    {
        public:
            using Ball = ball_interfaces::action::Ball;
            using GoalHandleBall = rclcpp_action::ClientGoalHandle<Ball>;

            explicit BallActionClient(const rclcpp::NodeOptions & options) : Node("ball_action_client", options)
            {
                RCLCPP_INFO(this->get_logger(), "Constructor Called");
                this->client_ptr_ = rclcpp_action::create_client<Ball>(this, "ball");
                this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&BallActionClient::send_goal, this));
            }

            void send_goal()
            {
                using namespace std::placeholders;
                this->timer_->cancel();

                if (!this->client_ptr_->wait_for_action_server())
                {
                    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                    rclcpp::shutdown();
                }

                auto goal_msg = Ball::Goal();
                goal_msg.requested = 10;
                RCLCPP_INFO(this->get_logger(), "Sending goal");

                auto send_goal_options = rclcpp_action::Client<Ball>::SendGoalOptions();
                send_goal_options.goal_response_callback = std::bind(&BallActionClient::goal_response_callback, this, _1);
                send_goal_options.feedback_callback = std::bind(&BallActionClient::feedback_callback, this, _1, _2);
                send_goal_options.result_callback = std::bind(&BallActionClient::result_callback, this, _1);
                this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
            }
        private:
            rclcpp_action::Client<Ball>::SharedPtr client_ptr_;
            rclcpp::TimerBase::SharedPtr timer_;

            void goal_response_callback(std::shared_future<GoalHandleBall::SharedPtr> future)
            {
                auto goal_handle = future.get();
                if (!goal_handle)
                {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                }
            }

            void feedback_callback(GoalHandleBall::SharedPtr, const std::shared_ptr<const Ball::Feedback> feedback)
            {
                std::stringstream ss;
                ss << "Waited for: ";
                ss << feedback->progress;
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            }

            void result_callback(const GoalHandleBall::WrappedResult & result)
            {
                switch (result.code)
                {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                        return;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Uknown result code");
                        return;
                }
                std::stringstream ss;
                ss << "Waited for: " << result.result->spun;
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
                rclcpp::shutdown();
            }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(delta::BallActionClient)