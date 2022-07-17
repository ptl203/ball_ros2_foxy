#include <functional>
#include <memory>
#include <thread>

#include "ball_interfaces/action/ball.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "ball_action/visibility_control.h"

namespace ball_action
{
    class BallActionServer : public rclcpp::Node
    {
        public:
            using Ball = ball_interfaces::action::Ball;
            using GoalHandleBall = rclcpp_action::ServerGoalHandle<Ball>;

            BALL_ACTION_PUBLIC
            explicit BallActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("ball_action_server", options)
            {
                using namespace std::placeholders;
                this->action_server_ = rclcpp_action::create_server<Ball> (
                    this,
                    "ball",
                    std::bind(&BallActionServer::handle_goal, this, _1, _2),
                    std::bind(&BallActionServer::handle_cancel, this, _1),
                    std::bind(&BallActionServer::handle_accepted, this, _1));
            }
        private:
            rclcpp_action::Server<Ball>::SharedPtr action_server_;
            rclcpp_action::GoalResponse handle_goal(
                const rclcpp_action::GoalUUID & uuid,
                std::shared_ptr<const Ball::Goal> goal)
            {
                RCLCPP_INFO(this->get_logger(), "Received goal request with goal %d", goal->request);
                (void)uuid;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandleBall> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }

            void handle_accepted(const std::shared_ptr<GoalHandleBall> goal_handle)
            {
                using namespace std::placeholders;
                // this needs to return quickly to avoid blocking the executor, so spin up a new thread
                std::thread{std::bind(&BallActionServer::execute, this, _1), goal_handle}.detach();
            }

            void execute(const std::shared_ptr<GoalHandleBall> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Executing goal");
                rclcpp::Rate loop_rate(1);
                const auto goal = goal_handle->get_goal();
                auto feedback = std::make_shared<Ball::Feedback>();
                auto & complete = feedback->progress;
                auto result = std::make_shared<Ball::Result>();

                for (int i = 0; (i < goal->request) && rclcpp::ok(); ++i)
                {
                    if (goal_handle->is_canceling()) {
                        result->complete = complete;
                        goal_handle->canceled(result);
                        RCLCPP_INFO(this->get_logger(), "Goal canceled");
                        return;
                    }
                    // Update Sequence
                    ++complete;
                    //Publish Feedback
                    goal_handle->publish_feedback(feedback);
                    RCLCPP_INFO(this->get_logger(), "Publish feedback");

                    loop_rate.sleep();
                }

                if (rclcpp::ok()) 
                {
                    result->complete = complete;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                }
            }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(ball_action::BallActionServer)