#include "lx_operation/operations_handler.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<OperationsHandler>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}