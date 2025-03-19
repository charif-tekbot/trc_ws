#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "tekbot_description/srv/timer_control.hpp"
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class TimedCommandPublisher : public rclcpp::Node {
public:
    TimedCommandPublisher() : Node("timed_command_publisher"), is_active_(false) {
        // Déclaration des paramètres
        declare_parameter<std::string>("command_topic", "/cmd_vel");
        declare_parameter<std::string>("odom_topic", "/odometry/ground_truth");
        declare_parameter<double>("default_duration", 5.0);
        declare_parameter<double>("default_linear_speed", 0.5);
        declare_parameter<double>("default_angular_speed", 0.0);

        // Récupération des paramètres
        command_topic_ = get_parameter("command_topic").as_string();
        odom_topic_ = get_parameter("odom_topic").as_string();
        default_duration_ = get_parameter("default_duration").as_double();
        default_linear_speed_ = get_parameter("default_linear_speed").as_double();
        default_angular_speed_ = get_parameter("default_angular_speed").as_double();

        // Initialisation des éditeurs et abonnements
        command_pub_ = create_publisher<geometry_msgs::msg::Twist>(command_topic_, 10);
        // odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        //     odom_topic_, 10, std::bind(&TimedCommandPublisher::odomCallback, this, _1));
        
        // Service de commande
        trigger_service_ = create_service<tekbot_description::srv::TimerControl>(
            "trigger_command", std::bind(&TimedCommandPublisher::triggerCommandCallback, this, _1, _2));

        RCLCPP_INFO(get_logger(), "TimedCommandPublisher ready. Call '/trigger_command' to start a command.");
    }

private:
    // Publishers, subscribers et service
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<tekbot_description::srv::TimerControl>::SharedPtr trigger_service_;

    std::string command_topic_, odom_topic_;
    double default_duration_, default_linear_speed_, default_angular_speed_;
    bool is_active_;

    void triggerCommandCallback(const tekbot_description::srv::TimerControl::Request::SharedPtr req,
                                tekbot_description::srv::TimerControl::Response::SharedPtr res) {
        if (is_active_) {
            res->success = false;
            res->message = "A command is already active.";
            RCLCPP_WARN(get_logger(), "Cannot start a new command. One is already active.");
            return;
        }

        double duration = req->duration > 0 ? req->duration : default_duration_;
        double linear_speed = req->linear_speed != 0 ? req->linear_speed : default_linear_speed_;
        double angular_speed = req->angular_speed != 0 ? req->angular_speed : default_angular_speed_;
        sendCommand(duration, linear_speed, angular_speed);
        res->success = true;
        res->message = "Command executed successfully.";
    }

    void sendCommand(double duration, double linear_speed, double angular_speed) {
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = linear_speed;
        cmd.angular.z = angular_speed;

        auto rate = rclcpp::Rate(10); // Publier à 10 Hz
        RCLCPP_INFO(get_logger(), "Publishing command for %.2f seconds.", duration);

        auto start_time = now();
        is_active_ = true;

        while ((now() - start_time).seconds() < duration) {
            command_pub_->publish(cmd);
            rate.sleep();
        }

        stopCommand();
    }

    void stopCommand() {
        auto stop_cmd = geometry_msgs::msg::Twist();
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;

        command_pub_->publish(stop_cmd);
        RCLCPP_INFO(get_logger(), "Command stopped.");
        is_active_ = false;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimedCommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
