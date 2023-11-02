#include "lx_localization/localization.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<Localization>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}