#include "lx_status_relay/status_relay.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<StatusRelay>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}