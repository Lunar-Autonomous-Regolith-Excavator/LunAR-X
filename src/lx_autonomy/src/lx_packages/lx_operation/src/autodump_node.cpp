#include "lx_operation/autodump.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<AutoDump>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}