#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sist_alertatemp_volcan/msg/custom_msg.hpp"

class PublisherNode : public rclcpp::Node {
    public:
    PublisherNode() : Node("publisher_node") {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
        publisher_ = this->create_publisher<std_msgs::msg::String>("std_string_topic", qos_profile);
        custom_publisher_ = this->create_publisher<sist_alertatemp_volcan::msg::CustomMsg>("custom_topic", qos_profile);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PublisherNode::publish_messages,this));
    }

    void publish_messages() {
        auto custom_msg = sist_alertatemp_volcan::msg::CustomMsg();
        auto string_msg = std_msgs::msg::String();
        string_msg.data = "Lectura actual del sistema 1";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", string_msg.data.c_str());
        publisher_->publish(string_msg);
        custom_msg.estado = "Inactivo";
        custom_msg.densidad = 10.0;
        custom_msg.identificador = 1;
        RCLCPP_INFO(this->get_logger(), "Publicando lectura actual: estado='%s',densidad=%.2f, sensor#=%d", custom_msg.estado.c_str(), custom_msg.densidad,custom_msg.identificador);
        custom_publisher_->publish(custom_msg);
    }
    
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Publisher<sist_alertatemp_volcan::msg::CustomMsg>::SharedPtr custom_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}