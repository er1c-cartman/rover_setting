#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include <cmath>

class FourWheelKinematics : public rclcpp::Node
{
public:
    FourWheelKinematics()
        : Node("four_wheel_kinematics")
    {
        // Declare and retrieve parameters from the config file
        this->declare_parameter<double>("wheel_base", 0.44);
        this->declare_parameter<double>("track_width", 0.375);
        this->declare_parameter<double>("wheel_radius", 0.085);  // Updated radius
        this->declare_parameter<int>("profile_acceleration", 100);
        this->declare_parameter<int>("front_left_wheel", 2);
        this->declare_parameter<int>("front_right_wheel", 3);
        this->declare_parameter<int>("rear_left_wheel", 4);
        this->declare_parameter<int>("rear_right_wheel", 5);

        this->get_parameter("wheel_base", wheel_base_);
        this->get_parameter("track_width", track_width_);
        this->get_parameter("wheel_radius", wheel_radius_);
        this->get_parameter("profile_acceleration", profile_acceleration_);
        this->get_parameter("front_left_wheel", front_left_wheel_);
        this->get_parameter("front_right_wheel", front_right_wheel_);
        this->get_parameter("rear_left_wheel", rear_left_wheel_);
        this->get_parameter("rear_right_wheel", rear_right_wheel_);

        // Create publishers for each wheel with velocity control
        fl_wheel_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_velocity", 10);
        fr_wheel_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_velocity", 10);
        rl_wheel_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_velocity", 10);
        rr_wheel_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_velocity", 10);

        // Subscribe to cmd_vel
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&FourWheelKinematics::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = msg->linear.x;
        double omega = msg->angular.z;

        // Calculate wheel velocities (m/s)
        double vl = v - (omega * track_width_ / 2.0); // Left wheels
        double vr = v + (omega * track_width_ / 2.0); // Right wheels

        // Convert linear velocity to angular velocity (rad/s) for the wheels
        double angular_vl = vl / wheel_radius_;
        double angular_vr = vr / wheel_radius_;

        // Convert angular velocity to motor velocity unit (0.229 RPM per unit)
        int motor_vl = static_cast<int>((angular_vl * 60 / (2 * M_PI)) / 0.229);
        int motor_vr = static_cast<int>((angular_vr * 60 / (2 * M_PI)) / 0.229);

        // Clamp the motor velocities to the range [-1023, 1023]
        motor_vl = std::clamp(motor_vl, -1023, 1023);
        motor_vr = std::clamp(motor_vr, -1023, 1023);

        publishWheelVelocity(front_left_wheel_, motor_vl);
        publishWheelVelocity(front_right_wheel_, -motor_vr);
        publishWheelVelocity(rear_left_wheel_, motor_vl);
        publishWheelVelocity(rear_right_wheel_, -motor_vr);
    }

    void publishWheelVelocity(int id, int velocity)
    {
        auto msg = dynamixel_sdk_custom_interfaces::msg::SetPosition();
        msg.id = id;
        msg.position = velocity;
        fl_wheel_pub_->publish(msg);
    }

    double wheel_base_;
    double track_width_;
    double wheel_radius_;
    int profile_acceleration_;
    int front_left_wheel_, front_right_wheel_, rear_left_wheel_, rear_right_wheel_;

    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr fl_wheel_pub_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr fr_wheel_pub_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr rl_wheel_pub_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr rr_wheel_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FourWheelKinematics>());
    rclcpp::shutdown();
    return 0;
}





// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
// #include <cmath>

// class FourWheelKinematics : public rclcpp::Node
// {
// public:
//     FourWheelKinematics()
//         : Node("four_wheel_kinematics")
//     {
//         // Declare parameters for wheel configuration
//         this->declare_parameter<double>("wheel_base", 0.44);
//         this->declare_parameter<double>("track_width", 0.375);
//         this->declare_parameter<double>("wheel_radius", 0.17);  // Wheel radius in meters
//         this->declare_parameter<int>("front_left_wheel", 2);
//         this->declare_parameter<int>("front_right_wheel", 3);
//         this->declare_parameter<int>("rear_left_wheel", 4);
//         this->declare_parameter<int>("rear_right_wheel", 5);

//         // Retrieve parameters
//         this->get_parameter("wheel_base", wheel_base_);
//         this->get_parameter("track_width", track_width_);
//         this->get_parameter("wheel_radius", wheel_radius_);
//         this->get_parameter("front_left_wheel", front_left_wheel_);
//         this->get_parameter("front_right_wheel", front_right_wheel_);
//         this->get_parameter("rear_left_wheel", rear_left_wheel_);
//         this->get_parameter("rear_right_wheel", rear_right_wheel_);

//         // Create publishers for each wheel
//         fl_wheel_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_velocity", 10);
//         fr_wheel_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_velocity", 10);
//         rl_wheel_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_velocity", 10);
//         rr_wheel_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("/set_velocity", 10);

//         // Subscribe to cmd_vel
//         cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "/cmd_vel", 10, std::bind(&FourWheelKinematics::cmdVelCallback, this, std::placeholders::_1));
//     }

// private:
//     void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
//     {
//         double v = msg->linear.x;
//         double omega = msg->angular.z;

//         // Calculate wheel velocities (m/s)
//         double vl = v - (omega * track_width_ / 2.0); // Left wheels
//         double vr = v + (omega * track_width_ / 2.0); // Right wheels

//         // Convert linear velocity to angular velocity (rad/s) for the wheels
//         double angular_vl = vl / wheel_radius_;
//         double angular_vr = vr / wheel_radius_;

//         // Convert angular velocity to motor velocity unit (value range 0 to 1023, where 0.229 RPM = 1 unit)
//         // angular velocity (rad/s) to RPM: angular_velocity * 60 / (2 * M_PI)
//         int motor_vl = static_cast<int>((angular_vl * 60 / (2 * M_PI)) / 0.229);
//         int motor_vr = static_cast<int>((angular_vr * 60 / (2 * M_PI)) / 0.229);

//         // Ensure the motor velocity does not exceed the allowed range
//         motor_vl = std::clamp(motor_vl, -1023, 1023);
//         motor_vr = std::clamp(motor_vr, -1023, 1023);

//         publishWheelVelocity(front_left_wheel_, motor_vl);
//         publishWheelVelocity(front_right_wheel_, -motor_vr);
//         publishWheelVelocity(rear_left_wheel_, motor_vl);
//         publishWheelVelocity(rear_right_wheel_, -motor_vr);
//     }

//     void publishWheelVelocity(int id, int velocity)
//     {
//         auto msg = dynamixel_sdk_custom_interfaces::msg::SetPosition();
//         msg.id = id;
//         msg.position = velocity;
//         fl_wheel_pub_->publish(msg);
//     }

//     double wheel_base_;
//     double track_width_;
//     double wheel_radius_;
//     int front_left_wheel_, front_right_wheel_, rear_left_wheel_, rear_right_wheel_;

//     rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr fl_wheel_pub_;
//     rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr fr_wheel_pub_;
//     rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr rl_wheel_pub_;
//     rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr rr_wheel_pub_;

//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<FourWheelKinematics>());
//     rclcpp::shutdown();
//     return 0;
// }
