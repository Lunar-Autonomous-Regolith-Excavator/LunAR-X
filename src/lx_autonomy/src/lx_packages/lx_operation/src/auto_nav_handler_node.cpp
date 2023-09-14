#include "lx_operation/auto_nav_handler.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<AutoNavHandler>(rclcpp::NodeOptions());
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}