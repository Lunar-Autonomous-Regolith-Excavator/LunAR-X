#include "lx_bringup_autonomy/lx_param_server.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<ParamServer>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}