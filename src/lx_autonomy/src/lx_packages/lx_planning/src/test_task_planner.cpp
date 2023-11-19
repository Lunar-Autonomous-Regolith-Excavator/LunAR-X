#include "rclcpp/rclcpp.hpp"
#include "lx_msgs/srv/plan.hpp"
#include "lx_msgs/msg/planned_task.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <vector>

class TestPlannerNode : public rclcpp::Node
{
public:
    TestPlannerNode() : Node("testPlanner")
    {
        client_ = create_client<lx_msgs::srv::Plan>("plan_operation");

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(11))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
        }

        // Call the service
        auto request = std::make_shared<lx_msgs::srv::Plan::Request>();
        // Fill in your request data here
        std::vector<geometry_msgs::msg::Point> berm_pts;
        geometry_msgs::msg::Point pt;
        pt.y = 0.5; pt.x = 5.619; berm_pts.push_back(pt);
        pt.y = 0.889; pt.x = 5.712; berm_pts.push_back(pt);
        pt.y = 1.281; pt.x = 5.793; berm_pts.push_back(pt);
        pt.y = 1.675; pt.x = 5.862; berm_pts.push_back(pt);
        pt.y = 2.072; pt.x = 5.915; berm_pts.push_back(pt);
        pt.y = 2.47; pt.x = 5.956; berm_pts.push_back(pt);
        pt.y = 2.869; pt.x = 5.983; berm_pts.push_back(pt);
        pt.y = 3.269; pt.x = 5.998; berm_pts.push_back(pt);
        pt.y = 3.669; pt.x = 6.0; berm_pts.push_back(pt);
        pt.y = 4.069; pt.x = 5.987; berm_pts.push_back(pt);
        pt.y = 4.468; pt.x = 5.961; berm_pts.push_back(pt);
        pt.y = 4.866; pt.x = 5.922; berm_pts.push_back(pt);
        pt.y = 5.263; pt.x = 5.87; berm_pts.push_back(pt);
        pt.y = 5.658; pt.x = 5.804; berm_pts.push_back(pt);
        pt.y = 6.05; pt.x = 5.726; berm_pts.push_back(pt);
        pt.y = 6.439; pt.x = 5.634; berm_pts.push_back(pt);

        request->berm_input = berm_pts;
        request->berm_height = 0.15;
        request->section_length = 0.4;


        auto result_future = client_->async_send_request(request);

        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            // Print out the berm sequence
            std::vector<lx_msgs::msg::PlannedTask> plan = result->plan;
            for (int i = 0; i < static_cast<int>(plan.size()); i++) {
                // Get yaw from quaternion
                double yaw = atan2(2.0 * (plan[i].pose.orientation.w * plan[i].pose.orientation.z + plan[i].pose.orientation.x * plan[i].pose.orientation.y),
                                   1.0 - 2.0 * (plan[i].pose.orientation.y * plan[i].pose.orientation.y + plan[i].pose.orientation.z * plan[i].pose.orientation.z));
                // Convert to degrees
                yaw = yaw * 180.0 / M_PI;
                RCLCPP_INFO(get_logger(), "%d, %f, %f, %f", plan[i].task_type, plan[i].pose.position.x, plan[i].pose.position.y, yaw);
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to call service");
        }
    }

private:
    rclcpp::Client<lx_msgs::srv::Plan>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPlannerNode>());
    rclcpp::shutdown();
    return 0;
}