#include "lx_mapping/pcl_relay.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<PCLRelay>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}