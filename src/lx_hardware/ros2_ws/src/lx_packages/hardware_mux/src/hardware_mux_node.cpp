#include "hardware_mux/hardware_mux.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareMux>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}