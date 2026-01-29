#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;


class Talker : public rclcpp::Node
{
public:
  Talker() : Node("talker")
  {
    // Declare parameters with default values
    this->declare_parameter<double>("v", 0.0);
    this->declare_parameter<double>("d", 0.0);
    
    // Publisher
    publisher_ = this->create_publisher<
    ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    // The 10 above says keep up to 10 outgoing messages buffered if they can’t be sent immediately.

    // Callback
    auto timer_callback =
        [this]() -> void {
            double v = this->get_parameter("v").as_double();
            double d = this->get_parameter("d").as_double();

            ackermann_msgs::msg::AckermannDriveStamped message;
            message.drive.speed = v;
            message.drive.steering_angle = d;
            this->publisher_->publish(message);
        };
    timer_ = this->create_wall_timer(std::chrono::nanoseconds(0), timer_callback);
  }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
