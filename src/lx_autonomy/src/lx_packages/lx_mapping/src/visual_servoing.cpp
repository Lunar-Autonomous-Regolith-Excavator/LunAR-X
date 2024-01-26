/* Author: Anish Senathi
 * Subscribers:
 *    - /tool_height: [StdMsgs/Float64] Height of the tool wrt base link
 *    - /camera/depth/color/points: [SensorMsgs/PointCloud2] Point cloud from the camera
 * Publishers:
 *    - /mapping/visual_servo_error: [GeometryMsgs/Point] Visual servo error
 *    - /mapping/C_peakline_marker: [VisualizationMsgs/Marker] Peak line marker
 *    - /mapping/D_targetpoint_marker: [VisualizationMsgs/Marker] Target point marker    
 *    - /mapping/D_transformed_berm_points: [VisualizationMsgs/Marker] Transformed berm points marker
 *    - /mapping/E_projected_point_marker: [VisualizationMsgs/Marker] Projected point marker
 *    - /mapping/B_peakpoints_marker: [VisualizationMsgs/Marker] Peak points marker
 *    - /mapping/A_groundplane_marker: [VisualizationMsgs/Marker] Ground plane marker
 *    - /mapping/A_bermplane_marker: [VisualizationMsgs/Marker] Berm plane marker
 *    - /mapping/groundplane: [SensorMsgs/PointCloud2] Ground plane point cloud
 *    - /mapping/bermplane: [SensorMsgs/PointCloud2] Berm plane point cloud
 * Services:
 *    - /mapping/visual_servo_switch: [LxMsgs/Switch] Service to start/stop visual servoing
 *
 * - Summary
 * - Performs visual servoing to align the tool with the berm using elevation thresholding
 * - Detects peak line and projects it onto the ground plane
 * - Calculates the error between the projected point and the target point for visual servoing
 * - Publishes the error and the markers for visualization* 
 * */

#include "lx_mapping/visual_servoing.hpp"

VisualServoing::VisualServoing() : Node("visual_servoing_node")
{   
    // Setup Communications
    setupCommunications();

    // When tool height not defined
    tool_height_wrt_base_link_ = 1000;

    // Initialize exp filters
    exp_filter_x_ = ExpFilter();
    exp_filter_y_ = ExpFilter();
    exp_filter_z_ = ExpFilter();

    RCLCPP_INFO(this->get_logger(), "Visual Servoing initialized");
}

void VisualServoing::setupCommunications(){
    // Subscribers
    tool_height_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("tool_height", 10, 
                                    std::bind(&VisualServoing::toolHeightCallback, this, std::placeholders::_1));
    tool_distance_subscriber_ = this->create_subscription<std_msgs::msg::Float64>("tool_distance", 10, 
                                    std::bind(&VisualServoing::toolDistanceCallback, this, std::placeholders::_1));
    // Publishers
    visual_servo_error_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("mapping/visual_servo_error", 10);
    peakline_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/C_peakline_marker", 10);
    targetpoint_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/D_targetpoint_marker", 10);
    if(debug_mode_){
        transformed_berm_points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/D_transformed_berm_points", 10);
        projected_point_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/E_projected_point_marker", 10);
        peakpoints_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/B_peakpoints_marker", 10);
        groundplane_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/A_groundplane_marker", 10);
        bermplane_marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("mapping/A_bermplane_marker", 10);
        groundplane_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/groundplane", 10);
        bermplane_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapping/bermplane", 10);
    }  

    // Servers
    visual_servo_switch_server_ = this->create_service<lx_msgs::srv::Switch>("mapping/visual_servo_switch", 
                        std::bind(&VisualServoing::startStopVSCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    // TF
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_, this, true);
}

void VisualServoing::toolHeightCallback(const std_msgs::msg::Float64::SharedPtr msg){
   tool_height_wrt_base_link_ = msg->data;
}

void VisualServoing::toolDistanceCallback(const std_msgs::msg::Float64::SharedPtr msg){
    tool_distance_wrt_base_link_ = msg->data;
}

void VisualServoing::startStopVSCallback(const std::shared_ptr<lx_msgs::srv::Switch::Request> req,
                                                std::shared_ptr<lx_msgs::srv::Switch::Response> res){
    RCLCPP_INFO(this->get_logger(), "Visual Servoing switch requested with state: %d", req->switch_state);
    if(req->switch_state ==true && node_state_ == false) // Turn on node if its ON requested and its not already ON
    {
        this->pointcloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("mapping/transformed_pointcloud", 10, 
                                        std::bind(&VisualServoing::pointCloudCallback, this, std::placeholders::_1));
        this->current_berm_segment = req->current_berm_segment;
        this->prev_berm_segment = req->prev_berm_segment;
        // print perv_berm_segment
        RCLCPP_INFO(this->get_logger(), "prev_berm_segment: x: %f, y: %f, theta: %f", prev_berm_segment.x, prev_berm_segment.y, prev_berm_segment.theta);
        // if all values in current_berm_segment are 0, set transform_mode_ to false
        if(this->current_berm_segment.x == 0 && this->current_berm_segment.y == 0 && this->current_berm_segment.theta == 0){
            transform_mode_ = false;
        }
        // else if (req->first_seg_dump == false) // only run transform points for first segment
        // {
        //     transform_mode_ = false;
        // }
        else if(prev_berm_segment.x < -900 && prev_berm_segment.y < -900 && prev_berm_segment.theta < -900) // if prev_berm_segment is not defined, set transform_mode_ to false
        {
            transform_mode_ = false;
        }
        else
        {
            transform_mode_ = true;
        }
        node_state_ = true;
    }
    else if (req->switch_state == false && node_state_ == true) // Turn off node if its OFF requested and its not already OFF
    {
        node_state_ = false;
        this->pointcloud_subscriber_.reset();
    }
    res->success = true;
}

void VisualServoing::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    pointcloud_thread_ = std::thread(std::bind(&VisualServoing::getVisualServoError, this, msg));

    pointcloud_thread_.detach();
}

double VisualServoing::getMedianElevation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    if(cloud->points.size() < 30){
        RCLCPP_INFO(this->get_logger(), "No berm, as could not find ground plane");
        return 0;
    }
    // bin the points in z direction to count the number of points in each bin
    std::vector<int> bin_values(NUM_BINS, 0); // vector of bin values
    for(long unsigned int i = 0; i < cloud->points.size(); i++){
        int bin_idx = int((cloud->points[i].z-PCL_Z_MIN_M)/(PCL_Z_MAX_M-PCL_Z_MIN_M)*NUM_BINS);
        if(bin_idx<0 || bin_idx>=NUM_BINS){
            continue;
        }
        bin_values[bin_idx]++;
    }
    // get the median elevation fof the points
    int median_bin_idx = cloud->points.size()/2;
    int sum = 0;
    for(int i = 0; i < NUM_BINS; i++){
        sum += bin_values[i];
        if(sum > median_bin_idx){
            median_bin_idx = i;
            break;
        }
    }
    double median_elevation = (double)median_bin_idx*(PCL_Z_MAX_M-PCL_Z_MIN_M)/NUM_BINS + PCL_Z_MIN_M;

    // if number of points more than median_elevation plus PEAK_LINE_DISTANCE_M, then no berm
    int num_points_above_threshold = 0;
    int threshold_bin_idx = int((median_elevation+PEAK_LINE_DISTANCE_M-PCL_Z_MIN_M)/(PCL_Z_MAX_M-PCL_Z_MIN_M)*NUM_BINS);
    // calculate using bins
    for(int i = threshold_bin_idx; i < NUM_BINS; i++){
        num_points_above_threshold += bin_values[i];
    }
    if(num_points_above_threshold < 10){
        RCLCPP_INFO(this->get_logger(), "No berm, as height of berm is too small");
        visual_servo_fail_ = true;
        return -100;
    }

    return median_elevation;
}

void VisualServoing::getGroundIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double median_elevation, pcl::PointIndices::Ptr ground_indices){
    // get the indices of the points in the ground plane using cropbox
    pcl::CropBox<pcl::PointXYZ> cropbox;
    cropbox.setMin(Eigen::Vector4f(PCL_X_MIN_M, PCL_Y_MIN_M, PCL_Z_MIN_M, 1.0));
    cropbox.setMax(Eigen::Vector4f(PCL_X_MAX_M, PCL_Y_MAX_M, median_elevation, 1.0));
    cropbox.setInputCloud(cloud);
    cropbox.filter(ground_indices->indices);
}


void VisualServoing::getBermIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double median_elevation, pcl::PointIndices::Ptr berm_indices){
    // get the indices of the points in the berm plane
    pcl::CropBox<pcl::PointXYZ> cropbox;
    cropbox.setMin(Eigen::Vector4f(PCL_X_MIN_M, PCL_Y_MIN_M, median_elevation, 1.0));
    cropbox.setMax(Eigen::Vector4f(PCL_X_MAX_M, PCL_Y_MAX_M, PCL_Z_MAX_M, 1.0));
    cropbox.setInputCloud(cloud);
    cropbox.filter(berm_indices->indices);
}

//function to apply RANSAC to fit ground plane on a pointcloud
pcl::PointIndices::Ptr VisualServoing::fitBestPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                                    int max_iterations, double distance_threshold, 
                                                    int round, pcl::PointIndices::Ptr inliers, 
                                                    pcl::ModelCoefficients::Ptr coefficients){
    (void)round;

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); // fit a plane
    seg.setMethodType (pcl::SAC_RANSAC); // use RANSAC
    seg.setMaxIterations (max_iterations); // set max iterations
    seg.setDistanceThreshold (distance_threshold); // set distance threshold
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    return inliers;
}

vector<double> VisualServoing::crossProduct(std::vector<double> v1, std::vector<double> v2){
    // cross product of 3 dimensional vectors
    vector<double> v3(3,0.0);
    v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
    return v3;
}

vector<double> VisualServoing::calculateNormalVector(pcl::ModelCoefficients::Ptr coefficients){
    // calculate normal vector from coefficients
    vector<double> normal_vector(3,0.0);
    normal_vector[0] = coefficients->values[0];
    normal_vector[1] = coefficients->values[1];
    normal_vector[2] = coefficients->values[2];
    // normalize it
    double norm = sqrt(normal_vector[0]*normal_vector[0] + normal_vector[1]*normal_vector[1] + normal_vector[2]*normal_vector[2]);
    normal_vector[0] = normal_vector[0]/norm;
    normal_vector[1] = normal_vector[1]/norm;
    normal_vector[2] = normal_vector[2]/norm;

    // make sure normal vector points upwards
    if(normal_vector[2] < 0){
        normal_vector[0] = -normal_vector[0];
        normal_vector[1] = -normal_vector[1];
        normal_vector[2] = -normal_vector[2];
    }
    return normal_vector;
}

void VisualServoing::publishVector(std::vector<double> v, std::string topic_name){
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = "base_link";
    marker_msg.header.stamp = this->get_clock()->now();
    marker_msg.ns = "visual_servoing";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::ARROW;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 0.05;
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.05;
    geometry_msgs::msg::Point p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    marker_msg.points.push_back(p);
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
    marker_msg.points.push_back(p);
    if(topic_name == "groundplane"){
        // red color
        marker_msg.color.r = 1.0;
        marker_msg.color.a = 1.0;
        if(debug_mode_){
            groundplane_marker_publisher_->publish(marker_msg);
        }
    }
    else if(topic_name == "bermplane"){
        // blue color
        marker_msg.color.b = 1.0;
        marker_msg.color.a = 1.0;
        if(debug_mode_){
            bermplane_marker_publisher_->publish(marker_msg);
        }
    }
    else if(topic_name == "targetpoint"){
        // green color
        marker_msg.color.g = 1.0;
        marker_msg.color.a = 1.0;
        marker_msg.points[0].x = tool_distance_wrt_base_link_;
        marker_msg.points[0].y = DRUM_Y_BASELINK_M;
        marker_msg.points[0].z = std::min(0.5, tool_height_wrt_base_link_) - DRUM_Z_BASELINK_M;
        targetpoint_marker_publisher_->publish(marker_msg);
    }
}

vector<double> VisualServoing::binPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, 
                                            pcl::PointIndices::Ptr inliers, 
                                            vector<double> ground_plane_equation){
    // make new pointcloud with only inliers
    // extract indices
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(in_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*inlier_cloud);

    std::vector<std::vector<std::vector<double>>> bin_values(NUM_BINS, std::vector<std::vector<double>>(NUM_BINS, std::vector<double>())); // 2D vector of bin values

    // bin the points
    for(long unsigned int i = 0; i < inlier_cloud->points.size(); i++){
        int row_idx = int((inlier_cloud->points[i].x-PCL_X_MIN_M)/(PCL_X_MAX_M-PCL_X_MIN_M)*NUM_BINS);
        int col_idx = int((inlier_cloud->points[i].y-PCL_Y_MIN_M)/(PCL_Y_MAX_M-PCL_Y_MIN_M)*NUM_BINS);
        if(row_idx<0 || row_idx>=NUM_BINS || col_idx<0 || col_idx>=NUM_BINS){
            continue;
        }
        bin_values[row_idx][col_idx].push_back(inlier_cloud->points[i].z);
    }

    std::vector<std::vector<double>> median_vec(NUM_BINS, std::vector<double>(NUM_BINS,-100.0)); // 2D vector of bin values
    for(int i = 0; i < NUM_BINS; i++){
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
    for(int j = 0; j < NUM_BINS; j++){
        double max_z = -100.0;
        for(int i = 0; i < NUM_BINS; i++){
                        if(median_vec[i][j]>max_z){
                max_z = median_vec[i][j];
                peak_x[j] = i;
                peak_z[j] = max_z;
            }
        }
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


    // Make a new PCL pointcloud with only the points in the bins
    pcl::PointCloud<pcl::PointXYZ>::Ptr binned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0; i < NUM_BINS; i++){
        pcl::PointXYZ p;
        p.x = ((double)peak_x[i]*(PCL_X_MAX_M-PCL_X_MIN_M)/NUM_BINS) + PCL_X_MIN_M;
        if (p.x < PCL_X_MIN_M + 0.01) continue;
        p.y = (double)i*(PCL_Y_MAX_M-PCL_Y_MIN_M)/NUM_BINS + PCL_Y_MIN_M;
        p.z = peak_z[i];
        binned_cloud->points.push_back(p);
    }

    // get the distance of the points in binned_cloud from the ground plane
    vector<double> distances;
    double max_distance = -100.0;
    for(long unsigned int i = 0; i < binned_cloud->points.size(); i++){
        double x = binned_cloud->points[i].x;
        double y = binned_cloud->points[i].y;
        double z = binned_cloud->points[i].z;
        double d = abs(ground_plane_equation[0]*x + ground_plane_equation[1]*y + ground_plane_equation[2]*z + ground_plane_equation[3])/sqrt(ground_plane_equation[0]*ground_plane_equation[0] + ground_plane_equation[1]*ground_plane_equation[1] + ground_plane_equation[2]*ground_plane_equation[2]);
        distances.push_back(d);
        if(d>max_distance){
            max_distance = d;
        }
    }

    // remove points that are not withing PEAK_LINE_DISTANCE_M of the ground plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr binned_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for(long unsigned int i=0; i < binned_cloud->points.size(); i++){
        if(abs(distances[i]-max_distance)<PEAK_LINE_DISTANCE_M){
            binned_cloud_filtered->points.push_back(binned_cloud->points[i]);
            geometry_msgs::msg::Point p;
            p.x = binned_cloud->points[i].x;
            p.y = binned_cloud->points[i].y;
            p.z = binned_cloud->points[i].z;
            marker_array_msg.points.push_back(p);
        }
    }
    if (binned_cloud_filtered->points.size() < 2){
        RCLCPP_INFO(this->get_logger(), "No berm, as could not find peak line");
        return {};
    }
    if(debug_mode_){
        peakpoints_marker_publisher_->publish(marker_array_msg);    
    }
    // fit a line to the binned cloud
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_peakline(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    // Set the model you want to fit (LINE for a best-fit line).
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);

    // Set the distance threshold for inliers (adjust as needed).
    seg.setDistanceThreshold(0.1); // Adjust this threshold according to your data.

    // Segment the largest line from the point cloud.
    seg.setInputCloud(binned_cloud_filtered);
    seg.segment(*inliers_peakline, *coefficients);

    // visualize these points as a line using LINE_STRIP
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header.frame_id = "base_link";
    marker_msg.header.stamp = this->get_clock()->now();
    marker_msg.ns = "visual_servoing";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 0.01;
    marker_msg.scale.y = 0.01;
    marker_msg.scale.z = 0.01;
    marker_msg.color.g = 1.0;
    marker_msg.color.a = 1.0;
    geometry_msgs::msg::Point p;
    p.x = coefficients->values[0] - coefficients->values[3];
    p.y = coefficients->values[1] - coefficients->values[4];
    p.z = coefficients->values[2] - coefficients->values[5];
    marker_msg.points.push_back(p);
    p.x = coefficients->values[0] + coefficients->values[3];
    p.y = coefficients->values[1] + coefficients->values[4];
    p.z = coefficients->values[2] + coefficients->values[5];
    marker_msg.points.push_back(p);
    peakline_marker_publisher_->publish(marker_msg);

    // vector to store line coefficients
    std::vector<double> line_coefficients;
    for(int i=0;i<6;i++){
        line_coefficients.push_back(coefficients->values[i]);
    }
    
    // In the inliers of the peak line, find the average x and y values
    double avg_x = 0.0, avg_y = 0.0;
    for(long unsigned int i = 0; i < inliers_peakline->indices.size(); i++){
        avg_x += binned_cloud_filtered->points[inliers_peakline->indices[i]].x;
        avg_y += binned_cloud_filtered->points[inliers_peakline->indices[i]].y;
    }
    avg_x = avg_x/inliers_peakline->indices.size();

    // append to line_coefficients
    line_coefficients.push_back(avg_x); line_coefficients.push_back(avg_y);

    return line_coefficients;
}

vector<geometry_msgs::msg::PoseStamped> VisualServoing::getTransformedBermSegments()
{
    // Convert current and previous berm segments to geometry_msgs::msg::PoseStamped 
    // Transform to base_link frame
    // Return vector of transformed berm segments

    // Read transform from map to base_link
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
        transformStamped = this->tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero, tf2::durationFromSec(0.5));
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        return {};
    }

    vector<geometry_msgs::msg::PoseStamped> ans;
    // transform berm segments
    tf2::Quaternion q;
    geometry_msgs::msg::PoseStamped curr_segment_pose;
    geometry_msgs::msg::PoseStamped prev_segment_pose;

    q.setRPY(0, 0, current_berm_segment.theta);
    curr_segment_pose.pose.position.x = current_berm_segment.x;
    curr_segment_pose.pose.position.y = current_berm_segment.y;
    curr_segment_pose.pose.position.z = 0.3; // for viz
    curr_segment_pose.pose.orientation.x = q.x(); curr_segment_pose.pose.orientation.y = q.y(); curr_segment_pose.pose.orientation.z = q.z(); curr_segment_pose.pose.orientation.w = q.w();

    q.setRPY(0, 0, prev_berm_segment.theta);
    prev_segment_pose.pose.position.x = prev_berm_segment.x;
    prev_segment_pose.pose.position.y = prev_berm_segment.y;
    prev_segment_pose.pose.position.z = 0.3; // for viz
    prev_segment_pose.pose.orientation.x = q.x(); prev_segment_pose.pose.orientation.y = q.y(); prev_segment_pose.pose.orientation.z = q.z(); prev_segment_pose.pose.orientation.w = q.w();

    geometry_msgs::msg::PoseStamped transformed_curr_segment_pose;
    geometry_msgs::msg::PoseStamped transformed_prev_segment_pose;

    tf2::doTransform(curr_segment_pose, transformed_curr_segment_pose, transformStamped);
    tf2::doTransform(prev_segment_pose, transformed_prev_segment_pose, transformStamped);

    ans.push_back(transformed_curr_segment_pose);
    ans.push_back(transformed_prev_segment_pose);

    return ans;
}

double VisualServoing::getTargetZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::CropBox<pcl::PointXYZ> crop;
    crop.setMin(Eigen::Vector4f(-10, -0.2, -10, 1.0));
    crop.setMax(Eigen::Vector4f(10, 0.2, 10, 1.0));
    crop.setInputCloud(cloud);
    crop.filter(*cropped_cloud);
    // reomove noise
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cropped_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cropped_cloud);

    double max_z = 0.0;
    std::for_each(cropped_cloud->points.begin(), cropped_cloud->points.end(), [&max_z](pcl::PointXYZ p){if(p.z>max_z){max_z = p.z;}});
    return max_z;
}

void VisualServoing::getVisualServoError(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

    visual_servo_fail_ = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_minus_plane1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients_1(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_1(new pcl::PointIndices);
    std::vector<double> line_coefficients;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane2(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::msg::PointCloud2 groundplane_msg;
    sensor_msgs::msg::PointCloud2 bermplane_msg;
    std::vector<double> ground_plane_equation;

    if(!USE_MEDIAN_SEGMENTATION){
        fitBestPlane(input_cloud, 100, 0.01, 1, inliers_1, coefficients_1);
        // delete inliers from cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(input_cloud);
        extract.setIndices(inliers_1);
        extract.setNegative(true);
        extract.filter(*cloud_minus_plane1);

        // if no points in cloud_minus_plane1, then no berm
        if(cloud_minus_plane1->points.size() < 30){
            RCLCPP_INFO(this->get_logger(), "No berm, as could not find second plane");
            visual_servo_fail_ = true;
            return;
        }
        pcl::ModelCoefficients::Ptr coefficients_2(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_2(new pcl::PointIndices);
        fitBestPlane(cloud_minus_plane1, 100, 0.01, 2, inliers_2, coefficients_2);

        std::vector<double> normal_1 = calculateNormalVector(coefficients_1);
        std::vector<double> normal_2 = calculateNormalVector(coefficients_2);
        auto cross_product = crossProduct(normal_1, normal_2);
        auto sin_theta = sqrt(cross_product[0]*cross_product[0] + cross_product[1]*cross_product[1] + cross_product[2]*cross_product[2]);
        auto theta = asin(sin_theta);
        
        // create 2 vector pointers
        std::vector<double> *ground_plane_vec = &normal_1;
        std::vector<double> *berm_plane_vec = &normal_2;    

        if(debug_mode_){
            pcl::ExtractIndices<pcl::PointXYZ> extract_d1, extract_d2;

            extract_d1.setInputCloud(input_cloud);
            extract_d1.setIndices(inliers_1);
            extract_d1.setNegative(false);
            extract_d1.filter(*cloud_plane1);
            groundplane_msg.header.frame_id = "base_link";

            extract_d2.setInputCloud(cloud_minus_plane1);
            extract_d2.setIndices(inliers_2);
            extract_d2.setNegative(false);
            extract_d2.filter(*cloud_plane2);
            bermplane_msg.header.frame_id = "base_link";
        }

        if (theta < MIN_PLANE_ANGLE_DEG*M_PI/180.0){
            // no berm
            ground_plane_vec = &normal_1;
            berm_plane_vec = &normal_1;
            RCLCPP_INFO(this->get_logger(), "No berm, as angle between planes is too small %f", theta);
            // publish ground plane
            if(debug_mode_){
                pcl::toROSMsg(*cloud_plane1, groundplane_msg);
                groundplane_publisher_->publish(groundplane_msg);
                pcl::toROSMsg(*cloud_plane2, bermplane_msg);
                bermplane_publisher_->publish(bermplane_msg);
            }
            visual_servo_fail_ = true;
        }
        else if(cross_product[1]<0){
            ground_plane_vec = &normal_1;
            berm_plane_vec = &normal_2;
            ground_plane_equation = {coefficients_1->values[0], coefficients_1->values[1], coefficients_1->values[2], coefficients_1->values[3]};
            line_coefficients = binPoints(cloud_minus_plane1, inliers_2, ground_plane_equation);
            if(debug_mode_)
            {
                pcl::toROSMsg(*cloud_plane1, groundplane_msg);
                groundplane_publisher_->publish(groundplane_msg);
                pcl::toROSMsg(*cloud_plane2, bermplane_msg);
                bermplane_publisher_->publish(bermplane_msg);
            }
        }
        else{
            ground_plane_vec = &normal_2;
            berm_plane_vec = &normal_1;
            ground_plane_equation = {coefficients_2->values[0], coefficients_2->values[1], coefficients_2->values[2], coefficients_2->values[3]};
            line_coefficients = binPoints(input_cloud, inliers_1, ground_plane_equation);
            if(debug_mode_)
            {
                pcl::toROSMsg(*cloud_plane2, groundplane_msg);
                groundplane_publisher_->publish(groundplane_msg);
                pcl::toROSMsg(*cloud_plane1, bermplane_msg);
                bermplane_publisher_->publish(bermplane_msg);
            }    
        }
        publishVector(*ground_plane_vec, "groundplane");
        publishVector(*berm_plane_vec, "bermplane");
    }
    else{
        // get median elevation
        double median_elevation = getMedianElevation(input_cloud);
        // get ground indices
        pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices);
        getGroundIndices(input_cloud, median_elevation, ground_indices);
        // get berm indices
        pcl::PointIndices::Ptr berm_indices(new pcl::PointIndices);
        getBermIndices(input_cloud, median_elevation, berm_indices);

        // if no points in berms, then no berm
        if(berm_indices->indices.size() < 30){
            RCLCPP_INFO(this->get_logger(), "No berm, as no points in berm indices");
            visual_servo_fail_ = true;
            return;
        }
        
        // fit plane thorugh ground indices
        pcl::ExtractIndices<pcl::PointXYZ> extract_d1;
        extract_d1.setInputCloud(input_cloud);
        extract_d1.setIndices(ground_indices);
        extract_d1.setNegative(false);
        extract_d1.filter(*cloud_plane1);
        groundplane_msg.header.frame_id = "base_link";
        pcl::PointIndices::Ptr inliers_1(new pcl::PointIndices);
        fitBestPlane(cloud_plane1, 100, 0.01, 1, inliers_1, coefficients_1);
        if(debug_mode_){
            // RCLCPP_INFO(this->get_logger(), "Ground plane coefficients: %f, %f, %f, %f", coefficients_1->values[0], coefficients_1->values[1], coefficients_1->values[2], coefficients_1->values[3]);
            // RCLCPP_INFO(this->get_logger(), "cloud_plane1 size: %ld", cloud_plane1->points.size());
        }
        // get ground plane equation
        ground_plane_equation = {coefficients_1->values[0], coefficients_1->values[1], coefficients_1->values[2], coefficients_1->values[3]};

        // call binPoints to get line coefficients
        line_coefficients = binPoints(input_cloud, berm_indices, ground_plane_equation);
        if(line_coefficients.size() == 0){
            RCLCPP_INFO(this->get_logger(), "No berm, as could not find peak line");
            visual_servo_fail_ = true;
            return;
        }
        if(debug_mode_)
        {   
            // cloud_plane2 is berm plane
            pcl::ExtractIndices<pcl::PointXYZ> extract_d2;
            extract_d2.setInputCloud(input_cloud);
            extract_d2.setIndices(berm_indices);
            extract_d2.setNegative(false);
            extract_d2.filter(*cloud_plane2);
            bermplane_msg.header.frame_id = "base_link";
            pcl::toROSMsg(*cloud_plane1, groundplane_msg);
            groundplane_publisher_->publish(groundplane_msg);
            pcl::toROSMsg(*cloud_plane2, bermplane_msg);
            bermplane_publisher_->publish(bermplane_msg);
        }
    }        
    
    // check if line_coefficients is empty
    if(line_coefficients.size() > 0){

        geometry_msgs::msg::Point error_msg, curr_error;
        std::vector<double> target_point(3,0.0);
        // find the point on the line given by line_coefficients that is closest to the tool (origin). 
        // The coefficients are the form (x0, y0, z0, x1, y1, z1, xmid, ymid)
        // where (x0, y0, z0) is a point on the line and (x1, y1, z1) is the direction vector of the line
        // (xmid, ymid) is the midpoint of the inliers of the peak line
        double t = -(line_coefficients[0]*line_coefficients[3] + line_coefficients[1]*line_coefficients[4] + line_coefficients[2]*line_coefficients[5])/
                        (line_coefficients[3]*line_coefficients[3] + line_coefficients[4]*line_coefficients[4] + line_coefficients[5]*line_coefficients[5]);
        target_point[0] = line_coefficients[0] + line_coefficients[3]*t;  // x
        target_point[1] = line_coefficients[1] + line_coefficients[4]*t;  // y
        target_point[2] = line_coefficients[2] + line_coefficients[5]*t;  // z

        // Get transformed berm segments
        vector<geometry_msgs::msg::PoseStamped> transformed_berm_segments = this->getTransformedBermSegments();
        bool got_transformed_berm_segments = true;
        if (transformed_berm_segments.size() == 0)
        {
            got_transformed_berm_segments = false;
            RCLCPP_INFO(this->get_logger(), "Visual Servoing could not transform berm segments, resorting to default behavior");
        }
        geometry_msgs::msg::PoseStamped curr_segment_pose, prev_segment_pose;
        double dist_to_curr_segment, dist_to_prev_segment;
        if(got_transformed_berm_segments == false)
        {
            dist_to_prev_segment = 1000; dist_to_curr_segment = 0; //to make sure curr_segment is chosen
        }
        else
        {
            curr_segment_pose = transformed_berm_segments[0];
            prev_segment_pose = transformed_berm_segments[1];
            double x_inlier = line_coefficients[6], y_inlier = line_coefficients[7]; // midpoint of the inliers of the peak line
            dist_to_curr_segment = sqrt(pow(x_inlier - curr_segment_pose.pose.position.x, 2) + pow(y_inlier - curr_segment_pose.pose.position.y, 2));
            dist_to_prev_segment = sqrt(pow(x_inlier - prev_segment_pose.pose.position.x, 2) + pow(y_inlier - prev_segment_pose.pose.position.y, 2));
            // print distance to curr and prev segments
            RCLCPP_INFO(this->get_logger(), "Distance to curr segment: %f, Distance to prev segment: %f", dist_to_curr_segment, dist_to_prev_segment);
        }

        if(transform_mode_ == false)
        {
            RCLCPP_INFO(this->get_logger(), "Transform mode is false, resorting to default behavior");
        }

        if(dist_to_prev_segment>dist_to_curr_segment || transform_mode_ == false)
        {
            // calculate yaw error by projecting the direction vector into the x-y plane
            double yaw_error = atan2(line_coefficients[3], line_coefficients[4]);
            // shift yaw error to -pi/2 to pi/2
            if(yaw_error > M_PI/2){
                yaw_error = yaw_error - M_PI;
            }
            else if(yaw_error < -M_PI/2){
                yaw_error = yaw_error + M_PI;
            }
            curr_error.x = target_point[0] - tool_distance_wrt_base_link_;
            curr_error.y = yaw_error;
            curr_error.z = target_point[2] - std::min(0.5, tool_height_wrt_base_link_) - DRUM_Z_BASELINK_M;
            RCLCPP_INFO(this->get_logger(), "Servoing to detected berm with errors: x: %f, y: %f, z: %f", curr_error.x, curr_error.y, curr_error.z);
            publishVector(target_point, "targetpoint");
        }
        else
        {
            // Detected berm is previous berm
            // Find closest point on line line_coefficients to the point previous_berm_segment
            vector<double> dir_vect = { line_coefficients[0] - prev_segment_pose.pose.position.x, 
                                        line_coefficients[1] - prev_segment_pose.pose.position.y, 
                                        line_coefficients[2] - prev_segment_pose.pose.position.z };

            double t = -(line_coefficients[3]*dir_vect[0] + line_coefficients[4]*dir_vect[1] + line_coefficients[5]*dir_vect[2])/
                        (line_coefficients[3]*line_coefficients[3] + line_coefficients[4]*line_coefficients[4] + line_coefficients[5]*line_coefficients[5]);

            vector<double> closest_point{3, 0.0};
            closest_point[0] = line_coefficients[0] + line_coefficients[3]*t;  // x
            closest_point[1] = line_coefficients[1] + line_coefficients[4]*t;  // y
            closest_point[2] = line_coefficients[2] + line_coefficients[5]*t;  // z

            // get theta of previous berm segment
            tf2::Quaternion q(prev_segment_pose.pose.orientation.x, prev_segment_pose.pose.orientation.y, prev_segment_pose.pose.orientation.z, prev_segment_pose.pose.orientation.w);
            double prev_segment_theta = tf2::getYaw(q);

            // get intersection point
            double SEG_LEN=GLOBAL_BERM_LENGTH_M;
            vector<double> intersection_point{3, 0.0};
            intersection_point[0] = closest_point[0] + cos(prev_segment_theta)*SEG_LEN/2.0;
            intersection_point[1] = closest_point[1] + sin(prev_segment_theta)*SEG_LEN/2.0;
            intersection_point[2] = closest_point[2];

            // get curr segment theta
            q = tf2::Quaternion(curr_segment_pose.pose.orientation.x, curr_segment_pose.pose.orientation.y, curr_segment_pose.pose.orientation.z, curr_segment_pose.pose.orientation.w);
            double curr_segment_theta = tf2::getYaw(q);

            // calculate target point
            vector<double> projected_point{3, 0.0};
            projected_point[0] = intersection_point[0] + cos(curr_segment_theta)*SEG_LEN/2.0;
            projected_point[1] = intersection_point[1] + sin(curr_segment_theta)*SEG_LEN/2.0;
            projected_point[2] = intersection_point[2];

            curr_error.x = projected_point[0] - tool_distance_wrt_base_link_;
            curr_error.y = M_PI/2 - curr_segment_theta;
            if (curr_error.y > M_PI/2){
                curr_error.y = curr_error.y - M_PI;
            }
            else if (curr_error.y < -M_PI/2){
                curr_error.y = curr_error.y + M_PI;
            }
            double z_approach = getTargetZ(input_cloud);
            curr_error.z = z_approach - std::min(0.5, tool_height_wrt_base_link_) - DRUM_Z_BASELINK_M;
            RCLCPP_INFO(this->get_logger(), "Previous berm segment is closer to target point, servoing with errors: x: %f, y: %f, z: %f", curr_error.x, curr_error.y, curr_error.z);
            publishVector(projected_point, "targetpoint");
            if(debug_mode_)
            {
                // projected_point_marker_publisher_ for intersection_point, projected_point and closest_point
                visualization_msgs::msg::Marker marker_msg;
                marker_msg.header.frame_id = "base_link";
                marker_msg.header.stamp = this->get_clock()->now();
                marker_msg.ns = "visual_servoing";
                marker_msg.id = 0;
                marker_msg.type = visualization_msgs::msg::Marker::POINTS;
                marker_msg.action = visualization_msgs::msg::Marker::ADD;
                marker_msg.pose.orientation.w = 1.0;
                marker_msg.scale.x = 0.05;
                marker_msg.scale.y = 0.05;
                marker_msg.color.g = 1.0;
                marker_msg.color.r = 1.0;
                marker_msg.color.a = 1.0;
                geometry_msgs::msg::Point p;
                p.x = intersection_point[0]; p.y = intersection_point[1]; p.z = intersection_point[2];
                marker_msg.points.push_back(p);
                p.x = projected_point[0]; p.y = projected_point[1]; p.z = projected_point[2];
                marker_msg.points.push_back(p);
                p.x = closest_point[0]; p.y = closest_point[1]; p.z = closest_point[2];
                marker_msg.points.push_back(p);
                projected_point_marker_publisher_->publish(marker_msg);

                // transformed_berm_points_publisher_ for prev_segment_pose and curr_segment_pose
                visualization_msgs::msg::Marker marker_msg2;
                marker_msg2.header.frame_id = "base_link";
                marker_msg2.header.stamp = this->get_clock()->now();
                marker_msg2.ns = "visual_servoing";
                marker_msg2.id = 0;
                marker_msg2.type = visualization_msgs::msg::Marker::POINTS;
                marker_msg2.action = visualization_msgs::msg::Marker::ADD;
                marker_msg2.pose.orientation.w = 1.0;
                marker_msg2.scale.x = 0.05;
                marker_msg2.scale.y = 0.05;
                marker_msg2.color.b = 1.0;
                marker_msg2.color.a = 1.0;
                p.x = prev_segment_pose.pose.position.x; p.y = prev_segment_pose.pose.position.y; p.z = prev_segment_pose.pose.position.z;
                marker_msg2.points.push_back(p);
                p.x = curr_segment_pose.pose.position.x; p.y = curr_segment_pose.pose.position.y; p.z = curr_segment_pose.pose.position.z;
                marker_msg2.points.push_back(p);
                transformed_berm_points_publisher_->publish(marker_msg2);
            }
        }

        // Filter errors 
        error_msg.x = exp_filter_x_.getValue(curr_error.x);
        error_msg.y = exp_filter_y_.getValue(curr_error.y);
        error_msg.z = exp_filter_z_.getValue(curr_error.z);
        visual_servo_error_publisher_->publish(error_msg);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No line coefficients");
        visual_servo_fail_ = true;
    }
}
