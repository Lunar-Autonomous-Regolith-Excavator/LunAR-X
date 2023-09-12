#include "lx_operation/autodig.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<AutoDig>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}