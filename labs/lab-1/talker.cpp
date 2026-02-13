#include <chrono> // For time durations
#include <memory> // For smart pointers

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ core API
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" // Ackermann steering message type

using namespace std::chrono_literals; // Easier syntax. E.g., `100ms` instead of `std::chrono::milliseconds(100)`

class Talker : public rclcpp::Node // All our node classes must inherit rclcpp::Node
{
public:
    Talker() : Node("talker") // Calling our base class (node) constructor, naming our node "Talker"
    {
        // Declare parameters with default values
        this->declare_parameter<double>("v", 0.0); // speed
        this->declare_parameter<double>("d", 0.0); // steering angle

        // Log startup message with initial parameters
        double v = this->get_parameter("v").as_double();
        double d = this->get_parameter("d").as_double();
        RCLCPP_INFO(this->get_logger(), "Talker node started with v=%.2f, d=%.2f", v, d);

        // Publisher: "drive" topic, AckermannDriveStamped type, queue size 10
        publisher_ = this->create_publisher<
        ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

        // Timer Callback: Lambda function which gets our parameters and publishes them to our drive topic
        auto timer_callback = 
            [this]() -> void {
                double v = this->get_parameter("v").as_double();
                double d = this->get_parameter("d").as_double();

                // Publisher
                ackermann_msgs::msg::AckermannDriveStamped msg;
                
                // Set header timestamp and optional frame_id
                msg.header.stamp = this->now(); // Current ROS2 time
                /* 
                Reference frame of the robot; tells other nodes what coordinate system this data belongs to
                E.g., If publishing sensor data instead, you might use 
                "laser_frame" or "camera_link" so nodes know where the data came from.
                */
                msg.header.frame_id = "base_link";

                msg.drive.speed = v;
                msg.drive.steering_angle = d;
                this->publisher_->publish(msg);
            };
        
        timer_ = this->create_wall_timer(100ms, timer_callback); // Runs our timer_callback every 100ms
    }
private:
    // ROS2 manages life cycle with shared ownership and this avoids memory leaks
    rclcpp::TimerBase::SharedPtr timer_; // Timer that triggers publishing
    // Publisher for Ackermann messages
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

// Entry point
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // Initialize ROS 2
    rclcpp::spin(std::make_shared<Talker>()); // Creates the node, keeps it alive, and processes callbacks
    rclcpp::shutdown(); // Cleans up ROS 2 properly
    return 0;
}

/*
ROS2 Design Notes

1) Parameters vs Variables
ROS parameters are runtime-configurable through the ROS2 system
(CLI, launch files, YAML). They are used for tunable behavior.
Normal C++ variables are internal and cannot be changed externally.

2) Shared Pointers
ROS2 uses std::shared_ptr for publishers, subscriptions, and timers
because execution is asynchronous. Multiple parts of the system may
share ownership, and shared_ptr ensures safe automatic memory cleanup.

3) `this->` Usage
`this` refers to the current class instance. While not always required,
`this->` is commonly used (especially inside lambdas) to clearly access
member variables and avoid scope ambiguity.

These patterns are standard in ROS2 C++ nodes.
*/
