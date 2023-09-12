#include "lx_operation/rover_navigate.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<RoverNavigate>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}