#include "lx_perception/berm_evaluation.hpp"

BermMap::BermMap() : Node("berm_evaluation_node")
{   
    // set false for dry runs, set true for printf commands and to publish occupancy grids
    debug_mode_ = false;

    //subscriber
    subscription_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera/depth/color/points", 10, std::bind(&BermMap::topic_callback_right, this, _1)); //subscribes to the point cloud topic at 1Hz

    // publishers for occupancy grids
    publisher_og_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_berm/occupancy_grid_2", 10);
    publisher_fil_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_berm/occupancy_grid_filtered", 10);
    
    // ros service to evaluate the berm
    service_ = create_service<BermMetrics>(
        "evaluate_berm",
        std::bind(&BermMap::service_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // set target berm height
    setTargetBermHeight(0.15);
    
    // configuring occupancy grid
    occupancy_grid_.header.frame_id = "base_link";
    occupancy_grid_.info.resolution = 0.05;
    occupancy_grid_.info.width = 200;
    occupancy_grid_.info.height = 200;
    occupancy_grid_.info.origin.position.x = -5;
    occupancy_grid_.info.origin.position.y = -5;
    occupancy_grid_.info.origin.position.z = 0;
    occupancy_grid_.info.origin.orientation.x = 0;
    occupancy_grid_.info.origin.orientation.y = 0;
    occupancy_grid_.info.origin.orientation.z = 0;
    occupancy_grid_.info.origin.orientation.w = 1;
    filtered_occupancy_grid_ = occupancy_grid_;
    berm_height_ = -1;
    berm_width_ = -1;
    berm_length_ = -1;
    is_initialized_left_ = false;
    is_initialized_right_ = false;
}

void BermMap::setTargetBermHeight(double height)
{
    berm_target_height_ = height;
}

// request: need_metrics = true, response: l,b,h & success
void BermMap::service_callback(const std::shared_ptr<BermMetrics::Request> request,
std::shared_ptr<BermMetrics::Response> response)
{
    response->success = process_right(msg_right_);
    response->height = berm_height_;
    response->width = berm_width_;
    response->length = berm_length_;
    RCLCPP_INFO(this->get_logger(), "Computed berm metrics: height: %f, width: %f, length: %f", response->height, response->width, response->length);
}

// recursive function to cluster cells with altitudes higher than 2cm to dunes by checking neighbours
void BermMap::add_dune_neighbors(std::vector<int> &dune_x, std::vector<int> &dune_y, std::vector<int> &dune_indices, int idx, int width)
{
    int x_idx = idx % width;
    int y_idx = idx / width;

    for (int i = x_idx - 1; i < x_idx + 2; i++)
    {
        for (int j = y_idx - 1; j < y_idx + 2; j++)
        {
            int candidate_idx = i + j * width;
            if (!(i == x_idx && j == y_idx) && occupancy_grid_.data[candidate_idx] > 20 && occupancy_grid_.data[candidate_idx]<100)
            {
                // check if i does not exist in dune_indices
                if (std::find(dune_indices.begin(), dune_indices.end(), candidate_idx) == dune_indices.end())
                {
                    dune_x.push_back(i);
                    dune_y.push_back(j);
                    dune_indices.push_back(candidate_idx);
                    add_dune_neighbors(dune_x, dune_y, dune_indices, candidate_idx, width);
                }
            }
        }
    }
}

// recursive function to get berm cells by checking neighbors, starting from peak
void BermMap::grow_dune(std::vector<int> &dune_indices,int &score, int idx, int width, int dune_counter, int rec_count)
{
    if(rec_count > 1000) return; // this is to prevent infinite recursion
    int x_idx = idx % width;
    int y_idx = idx / width;
    // filtered_occupancy_grid_.data[idx] = 0;

    for (int i = x_idx - 1; i < x_idx + 2; i++)
    {
        for (int j = y_idx - 1; j < y_idx + 2; j++)
        {
            int candidate_idx = i + j * width;
            if (!(i == x_idx && j == y_idx) && occupancy_grid_.data[candidate_idx] > 20 && occupancy_grid_.data[candidate_idx] < 100)
            {
                // check if i does not exist in dune_indices
                if (std::find(dune_indices.begin(), dune_indices.end(), candidate_idx) == dune_indices.end())
                {
                    dune_indices.push_back(candidate_idx);
                    score += occupancy_grid_.data[candidate_idx]*occupancy_grid_.data[candidate_idx];
                    grow_dune(dune_indices, score, candidate_idx, width, dune_counter, rec_count+1);
                }
            }
        }
    }
}

void BermMap::topic_callback_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if(!is_initialized_right_) is_initialized_right_ = true;
    msg_right_ = msg;
    // if(debug_mode_){
        bool success = process_right(msg_right_);
    // }
}

bool BermMap::process_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Error handling for pointcloud initialization
    if(!is_initialized_right_){
        RCLCPP_ERROR(this->get_logger(), "Error: Point cloud not initialized");
        return false;
    }

    // Error handling for pointcloud age
    // rclcpp::Time current_time = this->now();
    // int cloud_age = current_time.seconds() - (msg->header.stamp.sec + msg->header.stamp.nanosec/1e9);
    // if(cloud_age > 3){
    //     RCLCPP_ERROR(this->get_logger(), "Error: Point cloud is %d seconds old", cloud_age);
    //     return false;
    // }

    occupancy_grid_.header.stamp = msg->header.stamp;
    occupancy_grid_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height);


    pc_density_grid_right_.header = occupancy_grid_.header;
    pc_density_grid_right_.info = occupancy_grid_.info;
    pc_density_grid_right_.info.origin.position.z = 10;
    pc_density_grid_right_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height);

    // initialize values of pointcloud density as 1
    // instead of zero to avoid div by zero error
    for (int i = 0; i < (int)(occupancy_grid_.info.width * occupancy_grid_.info.height); i++)
    {
        int val = std::min(i % occupancy_grid_.info.height, occupancy_grid_.info.width - i % occupancy_grid_.info.height);
        pc_density_grid_right_.data[i] = 1;
    }

    const double number_of_points = msg->height * msg->width;

    int num_iterations = 100;
    double num_inliers = 0, max_num_inliers = 0;
    double a, b, c, d; // plane coefficients

    // RANSAC to fit ground plane
    for (size_t i = 0; i < num_iterations; i++)
    {

        sensor_msgs::PointCloud2Iterator<float> iter_x3(*msg, "x");
        // randomly select 3 indices in increasing order
        int idx1 = rand() % (int)number_of_points;
        int idx2 = rand() % (int)number_of_points;
        int idx3 = rand() % (int)number_of_points;
        if (idx1 > idx2)
        {int temp = idx1;idx1 = idx2;idx2 = temp;}
        if (idx2 > idx3)
        {int temp = idx2;idx2 = idx3;idx3 = temp;
        }
        if (idx1 > idx2)
        {int temp = idx1;idx1 = idx2;idx2 = temp;
        }

    double k = 0;
    double x1,y1,z1,x2,y2,z2,x3,y3,z3;
    for (sensor_msgs::PointCloud2Iterator<float> iter_x2(*msg, "x"); iter_x2 != iter_x2.end(); ++iter_x2, ++k){
        if(k == idx1){
            x1 = iter_x2[0];
            y1 = iter_x2[1];
            z1 = iter_x2[2];   
        }
        if(k == idx2){
            x2 = iter_x2[0];
            y2 = iter_x2[1];
            z2 = iter_x2[2];   
        }
        if(k == idx3){
            x3 = iter_x2[0];
            y3 = iter_x2[1];
            z3 = iter_x2[2];   
        }

    }
        // get coefficients of the plane
        double a1 = y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2);
        double b1 = z1 * (x2 - x3) + z2 * (x3 - x1) + z3 * (x1 - x2);
        double c1 = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
        double d1 = -(x1 * (y2 * z3 - y3 * z2) + x2 * (y3 * z1 - y1 * z3) + x3 * (y1 * z2 - y2 * z1));

        num_inliers = 0;

        for (size_t i = 0; i < number_of_points; ++i, ++iter_x3)
        {
            double x = iter_x3[0];
            double y = iter_x3[1];
            double z = iter_x3[2];
            double dist = fabs(a1 * x + b1 * y + c1 * z + d1) / sqrt(a1 * a1 + b1 * b1 + c1 * c1);
            if (dist < 0.01)
            {
                num_inliers++;
            }
        }
        if (num_inliers > max_num_inliers)
        {
            max_num_inliers = num_inliers;
            a = a1;b = b1;c = c1;d = d1;
        }
    }
    if (c < 0)
    { // make sure the normal vector is pointing upwards
        a = -a;b = -b;c = -c;d = -d;
    }
    double denom = sqrt(a * a + b * b + c * c);
    if (debug_mode_)
        printf("Ground plane: a = %f, b = %f, c = %f, d = %f, max_inliers = %f, num_points = %f\n", a, b, c, d, max_num_inliers, number_of_points);

    // pointcloud iterator to get rgb values
    // sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(*msg, "rgb");
    double min_z_val = 1000, max_z_val = -1000;
    double z_values[occupancy_grid_.info.width * occupancy_grid_.info.height];
    int berm_peak = -1;
    double berm_height_right = -1000;
    for (size_t i = 0; i < occupancy_grid_.info.width * occupancy_grid_.info.height; i++)
    {
        z_values[i] = 0;
    }

    double x_min = 1000, x_max = -1000, y_min = 1000, y_max = -1000;
    double height_density[50];
    for (size_t i = 0; i < 50; i++)
    {
        height_density[i] = 0;
    }

    for (sensor_msgs::PointCloud2Iterator<float> iter_x4(*msg, "x"); iter_x4 != iter_x4.end(); ++iter_x4)
    {
        double x = iter_x4[0];
        double y = iter_x4[1];
        double z = iter_x4[2];
        int x_idx = (int)(3 * x / occupancy_grid_.info.resolution) - 1.25 * occupancy_grid_.info.width / 2;
        int y_idx = (int)(5 * y / occupancy_grid_.info.resolution) + 1.25 * occupancy_grid_.info.height / 2;

        double z_val = -(a * x + b * y + c * z + d) / denom;
        int height_idx = (int)(z_val / 0.01);

        if (height_idx >= 0 && height_idx < 50)
        {
            height_density[height_idx]++;
        }
    }

    double z_val_threshold = 0.0;
    for(int l=0;l<50;l++){
        if(height_density[l]==0){
            z_val_threshold = l*0.01;
            break;
        }
    }
    if (z_val_threshold <=0.01){
        z_val_threshold = 0.02;
    }
    printf("z_val_threshold = %f\n", z_val_threshold);

    // for (sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x"); iter_x != iter_x.end(); ++iter_x, ++iter_rgb)
    for (sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x"); iter_x != iter_x.end(); ++iter_x)
    {
        double x = iter_x[0];
        double y = iter_x[1];
        double z = iter_x[2];
        // u_int8_t r = iter_rgb[0];
        // u_int8_t g = iter_rgb[1];
        int x_idx = (int)(3 * x / occupancy_grid_.info.resolution) - 1.25 * occupancy_grid_.info.width / 2;
        int y_idx = (int)(5 * y / occupancy_grid_.info.resolution) + 1.25 * occupancy_grid_.info.height / 2;

        double z_val = -(a * x + b * y + c * z + d) / denom;
        int idx = x_idx + y_idx * occupancy_grid_.info.width;
        // int hsv_val = std::max(r, g);
        // int hsv_val = std::max(std::max(r, g), b);

        if (idx >= 0 && idx < (int)(occupancy_grid_.info.width * occupancy_grid_.info.height))
        {
            // if (z_val < 0.1 && y>-0.5 && y<0.1)
            // if(hsv_val > 100 && z_val < z_val_threshold && y>-0.5)
            if(z_val < z_val_threshold && y>-0.5)
            { // if the point is on the ground and is regolith
                z_values[idx] += z_val;
                // z_values[idx] += hsv_val;

                pc_density_grid_right_.data[idx]++;
                if (z_val > berm_height_right)
                {
                    berm_height_right = z_val;
                    berm_peak = idx;
                }
            }
        }
        if (x < x_min)
        {
            x_min = x;
        }
        if (x > x_max)
        {
            x_max = x;
        }
        if (z_val < min_z_val)
        {
            min_z_val = z_val;
        }
        if (z_val > max_z_val)
        {
            max_z_val = z_val;
        }
    }

    // set the altitude values
    for (int i = 0; i < (int)(occupancy_grid_.info.width * occupancy_grid_.info.height); i++)
    {
        if (pc_density_grid_right_.data[i] > 0)
        {
            // linear mapping for visualization
            int height = (int) 1000 * (z_values[i] / pc_density_grid_right_.data[i]);
            // if(height > 0 && height < 100)
                occupancy_grid_.data[i] = height;
            if(height >z_val_threshold*1000 - 20){
                // printf("i=%d, height=%d", i, height);
                occupancy_grid_.data[i] = 99;
            }
        }
    }

    filtered_occupancy_grid_.header = occupancy_grid_.header;
    filtered_occupancy_grid_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height);
    filtered_occupancy_grid_.data = occupancy_grid_.data;

    if(debug_mode_){
        printf("x_min = %f, x_max = %f, y_min = %f, y_max = %f\n", x_min, x_max, y_min, y_max);
        printf("berm height = %f\n", berm_height_right);
        printf("berm peak = %d\n", berm_peak);
    }

    // segment all sand dunes
    std::vector<std::vector<int>> dunes_indices;
    std::vector<int> dunes_scores;
    int idx = 0;
    int dune_count = 10;
    int berm_idx = -1, berm_score = 0;
    while(idx!=filtered_occupancy_grid_.data.size())
    {
        // set idx to the next index of occupancy_grid_.data having value > 10
        idx = std::find_if(filtered_occupancy_grid_.data.begin()+idx+1, filtered_occupancy_grid_.data.end(), [](int i){return i>10;}) - filtered_occupancy_grid_.data.begin();
        std::vector<int> dune1_indices;
        int dune_score = 0;
        int rec_count = 0;
        grow_dune(dune1_indices, dune_score, idx, filtered_occupancy_grid_.info.width, dune_count, rec_count);
        dunes_indices.push_back(dune1_indices);
        dunes_scores.push_back(dune_score);
        // choose berm
        if (dune_score > berm_score)
        {
            berm_idx = dune_count/10-1;
            berm_score = dune_score;
        }
        dune_count+=    10;
    }
    int berm_height_max = 0;    
    int berm_peak2 = -1;
    for(int j=0; j<dunes_indices[berm_idx].size(); j++)
    {
        if(occupancy_grid_.data[dunes_indices[berm_idx][j]] > berm_height_max)
        {
            berm_height_max = occupancy_grid_.data[dunes_indices[berm_idx][j]];
            berm_peak2 = dunes_indices[berm_idx][j];
        }
        filtered_occupancy_grid_.data[dunes_indices[berm_idx][j]] = 100;
    }
    
    if(berm_peak2==-1){
        RCLCPP_ERROR(this->get_logger(), "Error: no berm peaks found");
        return false;
    }

    berm_peak = berm_peak2;

    std::vector<int> dune_indices_x;
    std::vector<int> dune_indices_y;
    std::vector<int> dune_indices;
    std::vector<int> peak_indices;

    std::cout<<"before add_dune_neighbors"<<std::endl;
    // accumulate x and y coordinates of occupancy_grid_ indices having value more than 20
    add_dune_neighbors(dune_indices_x, dune_indices_y, dune_indices, berm_peak, occupancy_grid_.info.width);
    std::cout<<"after add_dune_neighbors"<<std::endl;

    // given x and y coordinates of dune, find the best fit line
    double m, intercept;
    double std_dev;
    if (dune_indices_x.size() > 0)
    {
        std::vector<int> x = dune_indices_x;
        std::vector<int> y = dune_indices_y;
        double sum_x = std::accumulate(x.begin(), x.end(), 0.0);
        double sum_y = std::accumulate(y.begin(), y.end(), 0.0);
        double sum_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
        double sum_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
        double sum_yy = std::inner_product(y.begin(), y.end(), y.begin(), 0.0);
        double n = x.size();
        m = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
        intercept = (sum_y - m * sum_x) / n;
        if(debug_mode_)
            printf("m = %f, c = %f\n", m, intercept);
        double mean_x = sum_x / n;
        double mean_y = sum_y / n;
        double sum_err = 0;
        for (int i = 0; i < n; i++)
        {
            double err = y[i] - (m * x[i] + intercept);
            sum_err += err * err;
        }
        std_dev = sqrt(sum_err / n);
    }

    int x_start = -1;
    int x_end = -1;
    int best_x_start = 0;
    int best_x_end = 0;
    bool isCounting = false;

    // select the endpoints of best fit line to estimate berm length
    for (int i = 0; i < occupancy_grid_.info.width; i++)
    {
        int y = m * i + intercept;
        if (y >= 0 && y < occupancy_grid_.info.height && occupancy_grid_.data[i + y * occupancy_grid_.info.width] > 20)

        {
            if (!isCounting)
            {
                x_start = i;
                isCounting = true;
            }
        }
        else if (isCounting)
        {
            x_end = i;
            isCounting = false;
            if (x_end - x_start > best_x_end - best_x_start)
            {
                best_x_start = x_start;
                best_x_end = x_end;
            }
        }
    }
    for (int i = best_x_start; i < best_x_end; i++)
    {
        int y = m * i + intercept;
        // if (y >= 0 && y < occupancy_grid_.info.height)
        filtered_occupancy_grid_.data[i + y * occupancy_grid_.info.width] = 200;
    }

    // berm length = 2 x (distance between start and end points of bestfit line)
    berm_length_ = (best_x_end - best_x_start) * sqrt(1 + m * m) * 2;
    // berm width = 7 * standard deviation
    berm_width_ = std_dev / sqrt(1 + m * m) * 7.0;
    // berm height = altitude of peak / 8
    berm_height_ =  z_val_threshold*50 + occupancy_grid_.data[berm_peak2]/20.0;
    // if(debug_mode_){
        // printf("berm_height_ = %f, berm_length_ = %f, berm_width_ = %f\n", berm_height_, berm_length_, berm_width_);
        // printf("done filtering occupancy grid\n");
    // }

    filtered_occupancy_grid_.data[berm_peak2] = 101;

    // if(debug_mode_){
        publisher_og_->publish(occupancy_grid_);
        publisher_fil_->publish(filtered_occupancy_grid_);
    // }

    // reset vectors
    dune_indices_x.clear();
    dune_indices_y.clear();
    dune_indices.clear();
    peak_indices.clear();
    dunes_indices.clear();
    dunes_scores.clear();
    return true;
}