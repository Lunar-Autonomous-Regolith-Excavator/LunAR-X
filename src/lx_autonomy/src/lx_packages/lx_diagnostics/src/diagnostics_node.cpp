#include "lx_diagnostics/diagnostics.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<Diagnostics>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}