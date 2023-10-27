#include "lx_description/joint_viz.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<JointViz>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}