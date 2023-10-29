/* Author: Anish Senathi
 * Subscribers:
 *    - /topic: description
 * Publishers:
 *    - /topic: description
 * Services:
 *    - /name (type): description
 * Actions:
 *    - /name (type): description
 *
 * - Summary
 * 
 * TODO
 * - Add Documentation
 * - Test with rover
 * - Complete getVisualServoError
 * - Check compatibility with AutoDump
 * */

#include "lx_mapping/visual_servoing.hpp"


VisualServoing::VisualServoing() : Node("visual_servoing_node")
{   
    // Set false for dry runs, set true for commands and to publish debug data
    debug_mode_ = true;

    // Setup Communications
    setupCommunications();

    // When tool height not defined
    tool_height_wrt_base_link_ = 1000;

}

void VisualServoing::setupCommunications(){
    // Subscribers
    tool_height_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("tool_height", 10, 
                                    std::bind(&VisualServoing::toolHeightCallback, this, _1));
    // Publishers
    visual_servo_publisher_ = this->create_publisher<std_msgs::msg::Float32>("mapping/visual_servoing_error", 10);
    // Servers
    visual_servo_switch_server_ = this->create_service<lx_msgs::srv::Switch>("mapping/visual_servo_switch", 
                        std::bind(&VisualServoing::startStopVSCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void VisualServoing::toolHeightCallback(const std_msgs::msg::Float64::SharedPtr msg){
   tool_height_wrt_base_link_ = msg->data;
}

void VisualServoing::startStopVSCallback(const std::shared_ptr<lx_msgs::srv::Switch::Request> req,
                                                std::shared_ptr<lx_msgs::srv::Switch::Response> res){
    if(req->start){
        this->pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("mapping/transformed_pointcloud", 10, 
                                        std::bind(&VisualServoing::pointCloudCallback, this, _1));
    }
    else{
        this->pointcloud_subscriber_.reset();
    }
    res->success = true;
}

void VisualServoing::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    pointcloud_thread_ = std::thread(std::bind(&PointCloudHandler::getVisualServoError, this, msg));

    pointcloud_thread_.detach();
}

void VisualServoing::getVisualServoError(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud_target_berm(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cropped_cloud_target_berm);
    // 2D vector of length 80x200
    // define a vector
    // std::vector< std::vector<int> > vec(4, std::vector<int>(4));
    std::vector<double> vec(1,100);
    int resolution = 5;
    int num_rows = 200/resolution;
    int num_cols = 80/resolution;
    std::vector<std::vector<double>> row ((num_cols+1)*num_rows,vec);
    RCLCPP_INFO(this->get_logger(), "D");

    for(int i = 0; i < cropped_cloud_target_berm->points.size(); i++){
        int idx = int(num_rows*100*(cropped_cloud_target_berm->points[i].x+0.2)/resolution) + int((100*cropped_cloud_target_berm->points[i].z)/resolution);
        row[idx].push_back(cropped_cloud_target_berm->points[i].y);
    }
    RCLCPP_INFO(this->get_logger(), "E");

    std::vector<double> median_vec(num_cols*num_rows,0);
    std::vector<double> peak_y(num_cols,0);
    std::vector<double> peak_z(num_cols,0);
    RCLCPP_INFO(this->get_logger(), "A");

    for(int i=0;i<num_cols;i++){
        double max_elev = 1000;
        for(int j=0;j<num_rows;j++){
            int idx = num_rows*i+j;
            std::sort(row[idx].begin(), row[idx].end());
            median_vec[idx] = row[idx][row[idx].size()/2];
            if(median_vec[idx]<max_elev && row[idx].size()>1){
                max_elev = median_vec[idx];
                peak_z[i] = max_elev;
                peak_y[i] = j*resolution/100.0;
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "C");

    for(int i=0;i<num_cols;i++){
        RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f", i*resolution/100.0,peak_y[i],peak_z[i]);
    }

    sensor_msgs::msg::PointCloud2 result_msg;

    // publisher_pc_->publish(result_msg);
}
