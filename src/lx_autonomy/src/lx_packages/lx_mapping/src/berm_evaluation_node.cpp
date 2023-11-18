#include "lx_mapping/berm_evaluation.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    // Initialize node
    auto node = std::make_shared<BermEvaluation>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}