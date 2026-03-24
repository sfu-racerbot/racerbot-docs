#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <memory>
#include <cmath>
#include <algorithm>

class Safety : public rclcpp::Node
{
public:
    Safety() : rclcpp::Node("safety_node")
    {
        RCLCPP_INFO(this->get_logger(), "Safety node started");

        /// TODO: create ROS subscribers and publishers (you can look at lab 1 talker/relay to help with syntax)
        /// TODO: create publisher to send drive/brake commands
        /// TODO: subscribe to odometry to get speed
        /// TODO: subscribe to laser scan for obstacle detection
    }

private:
    static constexpr float TTC_THRESHOLD_ = 0.3f;
    float speed_ = 0.0; // current longitudinal speed
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

    // Updates speed_ to our current longitudinal velocity
    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
    }

    // Calculates each scan's iTTC and brakes the car if below the threshold for an imminent crash
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        // TTC = r / {-rdot}+ (but -rdot is +rdot for us because our range rate is positive when getting closer)
        // calculate iTTC
        /// TODO: loop through scan_msg->ranges
        /// TODO: skip if no object in proximity
        /// TODO: compute angle relative to front using angle_min and angle_increment
        /// TODO: compute range_rate = speed * cos(angle)
        /// TODO: compute closing_rate (TTC denominator) with negative rates set to zero
        /// TODO: skip if object is not approaching
        /// TODO: iTTC = range / closing_rate
        /// TODO: publish brake if iTTC <= TTC_THRESHOLD_
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
