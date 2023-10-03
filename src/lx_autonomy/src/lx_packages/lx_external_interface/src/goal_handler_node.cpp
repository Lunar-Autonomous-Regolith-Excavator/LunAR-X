#include "lx_external_interface/goal_handler.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<GoalHandler>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}