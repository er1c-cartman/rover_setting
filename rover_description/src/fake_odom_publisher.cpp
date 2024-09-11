#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>

using namespace std::chrono_literals;

class FakeOdomPublisher : public rclcpp::Node
{
public:
    FakeOdomPublisher()
    : Node("fake_odom_publisher"), x_(0.0), y_(0.0), theta_(0.0)
    {
        // Load parameters
        this->declare_parameter<double>("wheel_base", 0.44);
        this->declare_parameter<double>("track_width", 0.375);
        this->declare_parameter<double>("wheel_radius", 0.085);

        wheel_base_ = this->get_parameter("wheel_base").as_double();
        track_width_ = this->get_parameter("track_width").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();

        // Initialize publishers and subscribers
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&FakeOdomPublisher::cmdVelCallback, this, std::placeholders::_1));

        // Timer to periodically publish odometry
        timer_ = this->create_wall_timer(
            100ms, std::bind(&FakeOdomPublisher::publish_odometry, this));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Store the command velocities
        linear_velocity_ = msg->linear.x;
        angular_velocity_ = msg->angular.z;
    }

    void publish_odometry()
    {
        // Compute the change in position and orientation
        double delta_time = 0.1; // Assuming 10Hz update rate
        double delta_x = linear_velocity_ * delta_time;
        double delta_theta = angular_velocity_ * delta_time;

        x_ += delta_x * cos(theta_);
        y_ += delta_x * sin(theta_);
        theta_ += delta_theta;

        // Create quaternion from yaw
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta_);
        geometry_msgs::msg::Quaternion odom_quat;
        tf2::convert(quat, odom_quat);

        // Publish odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.197; // Set z position to base_link height from ground
        odom_msg.pose.pose.orientation = odom_quat;

        odom_msg.twist.twist.linear.x = linear_velocity_;
        odom_msg.twist.twist.angular.z = angular_velocity_;

        odom_publisher_->publish(odom_msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_, y_, theta_;
    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    double wheel_base_;
    double track_width_;
    double wheel_radius_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakeOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}

