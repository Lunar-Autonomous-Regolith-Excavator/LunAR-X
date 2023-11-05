#include "lx_mapping/world_model.hpp"

int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	// Initialize node
	auto node = std::make_shared<WorldModel>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}