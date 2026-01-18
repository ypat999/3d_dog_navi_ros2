#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserListener : public rclcpp::Node
{
public:
    LaserListener() : Node("laser_listener")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserListener::laser_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Laser listener node started");
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Received laser scan: %zu ranges, angle_min: %.2f, angle_max: %.2f, range_min: %.2f, range_max: %.2f",
            msg->ranges.size(), msg->angle_min, msg->angle_max, msg->range_min, msg->range_max);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserListener>());
    rclcpp::shutdown();
    return 0;
}
