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
                                    std::bind(&VisualServoing::toolHeightCallback, this, std::placeholders::_1));
    // Publishers
    visual_servo_publisher_ = this->create_publisher<std_msgs::msg::Float64>("mapping/visual_servoing_error", 10);
    visual_servo_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/visual_servoing_marker", 10);
    // Servers
    visual_servo_switch_server_ = this->create_service<lx_msgs::srv::Switch>("mapping/visual_servo_switch", 
                        std::bind(&VisualServoing::startStopVSCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void VisualServoing::toolHeightCallback(const std_msgs::msg::Float64::SharedPtr msg){
   tool_height_wrt_base_link_ = msg->data;
}

void VisualServoing::startStopVSCallback(const std::shared_ptr<lx_msgs::srv::Switch::Request> req,
                                                std::shared_ptr<lx_msgs::srv::Switch::Response> res){
    if(req->switch_state){
        this->pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("mapping/transformed_pointcloud", 10, 
                                        std::bind(&VisualServoing::pointCloudCallback, this, std::placeholders::_1));
    }
    else{
        this->pointcloud_subscriber_.reset();
    }
    res->success = true;
}

void VisualServoing::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    pointcloud_thread_ = std::thread(std::bind(&VisualServoing::getVisualServoError, this, msg));

    pointcloud_thread_.detach();
}

void VisualServoing::getVisualServoError(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);

    std::vector<std::vector<std::vector<double>>> bin_values(NUM_BINS, std::vector<std::vector<double>>(NUM_BINS, std::vector<double>())); // 2D vector of bin values

    RCLCPP_INFO(this->get_logger(), "D");
    // print cloud size
    RCLCPP_INFO(this->get_logger(), "cloud size: %d", input_cloud->points.size());

    for(int i = 0; i < input_cloud->points.size(); i++){
        int row_idx = int((input_cloud->points[i].x-PCL_X_MIN_M)/(PCL_X_MAX_M-PCL_X_MIN_M)*NUM_BINS);
        int col_idx = int((input_cloud->points[i].y-PCL_Y_MIN_M)/(PCL_Y_MAX_M-PCL_Y_MIN_M)*NUM_BINS);
        if(row_idx<0 || row_idx>=NUM_BINS || col_idx<0 || col_idx>=NUM_BINS){
            continue;
        }
        // bin_values[row_idx][col_idx].push_back(input_cloud->points[i].z);
        bin_values[row_idx][col_idx].push_back(input_cloud->points[i].z);
    }
    // print min and max values
    RCLCPP_INFO(this->get_logger(), "E");

    std::vector<std::vector<double>> median_vec(NUM_BINS, std::vector<double>(NUM_BINS,0.0)); // 2D vector of bin values
    for(int i=0;i<NUM_BINS;i++){
        for(int j=0;j<NUM_BINS;j++){
            // calculate median of bin using O(N) algorithm
            int bin_size = bin_values[i][j].size();
            if(bin_size<1){
                continue;
            }
            std::nth_element(bin_values[i][j].begin(), bin_values[i][j].begin() + bin_size/2, bin_values[i][j].end());
            median_vec[i][j] = bin_values[i][j][bin_size/2];
        }
    }

    // for every bin in y, find the x value with the highest z value
    std::vector<int> peak_x(NUM_BINS,0.0);
    std::vector<double> peak_z(NUM_BINS,0.0);
    for(int j=0;j<NUM_BINS;j++){
        double max_z = 0.0;
        for(int i=0;i<NUM_BINS;i++){
            if(median_vec[i][j]>max_z){
                max_z = median_vec[i][j];
                peak_x[j] = i;
                peak_z[j] = max_z;
            }
        }
    }

    for(int i=0;i<NUM_BINS;i++){
        RCLCPP_INFO(this->get_logger(), "peak_x[%d]: %d", i, peak_x[i]);
    }
    // visualization_msgs/MarkerArray.msg make
    visualization_msgs::msg::Marker marker_array_msg;
    // fill (peak_x[i], i, peak_z[i]) into marker_array_msg
    marker_array_msg.header.frame_id = "base_link";
    marker_array_msg.header.stamp = this->get_clock()->now();
    marker_array_msg.ns = "visual_servoing";
    marker_array_msg.id = 0;
    marker_array_msg.type = visualization_msgs::msg::Marker::POINTS;
    marker_array_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_array_msg.pose.orientation.w = 1.0;
    marker_array_msg.scale.x = 0.05;
    marker_array_msg.scale.y = 0.05;
    marker_array_msg.color.r = 1.0;
    marker_array_msg.color.a = 1.0;
    for(int i=0;i<NUM_BINS;i++){
        geometry_msgs::msg::Point p;
        p.x = ((double)peak_x[i]*(PCL_X_MAX_M-PCL_X_MIN_M)/NUM_BINS) + PCL_X_MIN_M;
        p.y = (double)i*(PCL_Y_MAX_M-PCL_Y_MIN_M)/NUM_BINS + PCL_Y_MIN_M;
        p.z = peak_z[i];
        marker_array_msg.points.push_back(p);
    }

   
    visual_servo_marker_publisher_->publish(marker_array_msg);    
}
