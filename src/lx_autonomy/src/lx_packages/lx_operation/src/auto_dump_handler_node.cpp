#include "lx_operation/auto_dump_handler.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<AutoDumpHandler>(rclcpp::NodeOptions());
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}