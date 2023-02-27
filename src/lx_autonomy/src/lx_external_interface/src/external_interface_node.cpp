#include "lx_external_interface/external_interface.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<ExternalInterface>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}