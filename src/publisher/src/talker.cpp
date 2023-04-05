#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"//allow us to use the most common pieces of the ROS2 system
#include "std_msgs/msg/string.hpp"//includes the built-in message type ypu will use to publish data

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node{
public:
    /*this publicconstructor names the node "minimal_publisher" and initializes "count_" to 0*/
    MinimalPublisher() : Node("minimal_publisher"), count_(0){
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        /*call timer_callback() twice a sec*/
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    /*the "timer_callback" is where the message data is set and the messages are actually published*/
    void timer_callback(){
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        /*the "RCLCPP" maro ensures every published message is printed to the console*/
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    /*declaration of the timer, publisher, and counter field*/
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);// ~ros::init
    rclcpp::spin(std::make_shared<MinimalPublisher>());//starts processing data from the node, including 
    rclcpp::shutdown();
    return 0;
}