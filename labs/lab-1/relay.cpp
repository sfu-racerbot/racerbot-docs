#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Relay : public rclcpp::Node
{
public:
    Relay() : Node("relay")
    {
        // Publisher
        publisher_ = this->create_publisher<
        ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);

        // Subscriber
        subscription_ = this->create_subscription<
        ackermann_msgs::msg::AckermannDriveStamped>(
        "drive", // topic name
        10, // QoS queue depth: Buffer up to 10 messages if the subscriber can’t keep up.
        [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) { // Callback
          ackermann_msgs::msg::AckermannDriveStamped out_msg;

          out_msg.drive.speed = msg->drive.speed * 3.0;
          out_msg.drive.steering_angle = msg->drive.steering_angle * 3.0;

          this->publisher_->publish(out_msg);
        });
    }

private:
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    };

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Relay>());
    rclcpp::shutdown();
    return 0;
}
