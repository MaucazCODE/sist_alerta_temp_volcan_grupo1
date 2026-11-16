#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sist_alertatemp_volcan/msg/custom_msg.hpp"


class SubscriberNode : public rclcpp::Node {
    public:
    SubscriberNode() : Node("subscriber_node") {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "std_string_topic", 
            qos_profile,
            std::bind(&SubscriberNode::string_callback, this,std::placeholders::_1));

        custom_subscription_ = this->create_subscription<sist_alertatemp_volcan::msg::CustomMsg>(
            "custom_topic", 
            qos_profile,
            std::bind(&SubscriberNode::custom_msg_callback, this,std::placeholders::_1));
    }

    void string_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    void custom_msg_callback(const sist_alertatemp_volcan::msg::CustomMsg::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Recibido mensaje: estado='%s',densidad=%.2f kg/m3, identificador=%d", msg->estado.c_str(), msg->densidad ,msg->identificador);
    }
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Subscription<sist_alertatemp_volcan::msg::CustomMsg>::SharedPtr custom_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}