#include "lx_planning/task_optimizer.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    // Initialize node
    auto node = std::make_shared<TaskOptimizer>();
    
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}