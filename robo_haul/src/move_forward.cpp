#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MoveForward : public rclcpp::Node
{
public:
    MoveForward() : Node("move_forward_node"), counter_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MoveForward::move_robot, this));

        RCLCPP_INFO(this->get_logger(), "Robot starting...");
    }

private:
    void move_robot()
    {
        geometry_msgs::msg::Twist msg;

        if (counter_ < 30) // 3 seconds (0.1 * 30)
        {
            msg.linear.x = 0.5; // forward
        }
        else
        {
            msg.linear.x = 0.0; // stop
        }

        publisher_->publish(msg);

        counter_++;
    }

    int counter_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveForward>());
    rclcpp::shutdown();
    return 0;
}