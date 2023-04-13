#include "lx_rover_command/command_mux.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<CommandMux>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}