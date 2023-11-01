#include "lx_mapping/visual_servoing.hpp"

int main(int argc, char **argv){
	rclcpp::init(argc, argv);

	// Initialize node
	auto node = std::make_shared<VisualServoing>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}