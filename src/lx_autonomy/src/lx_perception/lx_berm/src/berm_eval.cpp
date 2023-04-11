#include <cstdio>

#include "berm_map.hpp"

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BermMap>());
    rclcpp::shutdown();
    printf("hello world lx_berm package\n");
    return 0;
}
