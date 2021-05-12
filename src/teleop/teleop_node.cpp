#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std;
using std::placeholders::_1;

class TeleopNode : public rclcpp::Node {
    public:
        TeleopNode()
            : Node("teleop")
        {
            RCLCPP_INFO(this->get_logger(), "Node constructor");
            vel_topic = this->create_publisher<geometry_msgs::msg::Twist>("commands/velocity", 10);
            joy_topic = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&TeleopNode::callback, this, _1));
        }

    private:
        void callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = msg->axes[1] * 0.3; // 最高並進速度 0.3 [m/s]
            twist.angular.z = msg->axes[0] * 1.0; // 最高回転速度 1.0 [rad/s]

			vel_topic->publish(twist);
        }

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_topic;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_topic;
};


int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TeleopNode>());

    rclcpp::shutdown();

    return 0;
}
