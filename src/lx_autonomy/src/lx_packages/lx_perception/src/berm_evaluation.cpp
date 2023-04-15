#include "lx_perception/berm_evaluation.hpp"

BermMap::BermMap() : Node("berm_evaluation_node")
{
    debug_mode_ = false;
    subscription_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera2/depth/color/points", 1, std::bind(&BermMap::topic_callback_right, this, _1)); //subscribes to the point cloud topic at 1Hz
    subscription_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera1/depth/color/points", 1, std::bind(&BermMap::topic_callback_left, this, _1));

    if(debug_mode_){
        publisher_og_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_berm/occupancy_grid_2", 1);
        publisher_fil_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_berm/occupancy_grid__filtered", 1);
    }
    service_ = create_service<BermMetrics>(
        "evaluate_berm",
        std::bind(&BermMap::service_callback, this, std::placeholders::_1, std::placeholders::_2));

    
    setTargetBermHeight(0.15);
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

void BermMap::service_callback(const std::shared_ptr<BermMetrics::Request> request,
std::shared_ptr<BermMetrics::Response> response)
{
    response->success = process_left(msg_left_) && process_right(msg_right_);
    response->height = berm_height_;
    response->width = berm_width_;
    response->length = berm_length_;
    RCLCPP_INFO(this->get_logger(), "Computed berm metrics: height: %f, width: %f, length: %f", response->height, response->width, response->length);
}

void BermMap::add_dune_neighbors(std::vector<int> &dune_x, std::vector<int> &dune_y, std::vector<int> &dune_indices, int idx, int width)
{
    int x_idx = idx % width;
    int y_idx = idx / width;

    for (int i = x_idx - 1; i < x_idx + 2; i++)
    {
        for (int j = y_idx - 1; j < y_idx + 2; j++)
        {
            int candidate_idx = i + j * width;
            if (!(i == x_idx && j == y_idx) && occupancy_grid_.data[candidate_idx] > 20)
            {
                // check if i does not exist in dune_indices_x
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

void BermMap::grow_dune(std::vector<int> &dune_indices,int &score, int idx, int width, int dune_counter)
{
    int x_idx = idx % width;
    int y_idx = idx / width;
    filtered_occupancy_grid_.data[idx] = 0;

    for (int i = x_idx - 1; i < x_idx + 2; i++)
    {
        for (int j = y_idx - 1; j < y_idx + 2; j++)
        {
            int candidate_idx = i + j * width;
            if (!(i == x_idx && j == y_idx) && occupancy_grid_.data[candidate_idx] > 20)
            {
                // check if i does not exist in dune_indices_x
                if (std::find(dune_indices.begin(), dune_indices.end(), candidate_idx) == dune_indices.end())
                {
                    dune_indices.push_back(candidate_idx);
                    score += occupancy_grid_.data[candidate_idx]*occupancy_grid_.data[candidate_idx];
                    grow_dune(dune_indices, score, candidate_idx, width, dune_counter);
                }
            }
        }
    }
}

void BermMap::topic_callback_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if(!is_initialized_right_) is_initialized_right_ = true;
    msg_right_ = msg;
}

void BermMap::topic_callback_left(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if(!is_initialized_left_) is_initialized_left_ = true;
    msg_left_ = msg;
}

bool BermMap::process_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // Make occupancy grid
    if(!is_initialized_right_){
        RCLCPP_ERROR(this->get_logger(), "Error: Point cloud from right camera not initialized");
        return false;
    }
    rclcpp::Time current_time = this->now();
    int cloud_age = current_time.seconds() - (msg->header.stamp.sec + msg->header.stamp.nanosec/1e9);
    if(cloud_age > 3){
        RCLCPP_ERROR(this->get_logger(), "Error: Point cloud from right camera is %d seconds old", cloud_age);
        return false;
    }

    occupancy_grid_.header = msg->header;
    occupancy_grid_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height);

    pc_density_grid_right_.header = occupancy_grid_.header;
    pc_density_grid_right_.info = occupancy_grid_.info;
    pc_density_grid_right_.info.origin.position.z = 10;
    pc_density_grid_right_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height);

    for (int i = 0; i < (int)(occupancy_grid_.info.width * occupancy_grid_.info.height); i++)
    {
        int val = std::min(i % occupancy_grid_.info.height, occupancy_grid_.info.width - i % occupancy_grid_.info.height);
        pc_density_grid_right_.data[i] = 1;
    }

    const double number_of_points = msg->height * msg->width;

    int num_iterations = 100;
    double num_inliers = 0, max_num_inliers = 0;
    double a, b, c, d; // plane coefficients

    double max_x, max_y, max_z, min_x, min_y, min_z;
    for (size_t i = 0; i < num_iterations; i++)
    {

        sensor_msgs::PointCloud2Iterator<float> iter_x2(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y2(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z2(*msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_x3(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y3(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z3(*msg, "z");
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
        double x1 = *iter_x2;
        double y1 = *iter_y2;
        double z1 = *iter_z2;
        for (int i = idx1; i < idx2; i++)
        {
            iter_x2 += 1;
            iter_y2 += 1;
            iter_z2 += 1;
        }
        double x2 = *iter_x2;
        double y2 = *iter_y2;
        double z2 = *iter_z2;
        for (int i = idx2; i < idx3; i++)
        {
            iter_x2 += 1;
            iter_y2 += 1;
            iter_z2 += 1;
        }
        double x3 = *iter_x2;
        double y3 = *iter_y2;
        double z3 = *iter_z2;
        // get coefficients of the plane
        double a1 = y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2);
        double b1 = z1 * (x2 - x3) + z2 * (x3 - x1) + z3 * (x1 - x2);
        double c1 = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
        double d1 = -(x1 * (y2 * z3 - y3 * z2) + x2 * (y3 * z1 - y1 * z3) + x3 * (y1 * z2 - y2 * z1));

        num_inliers = 0;

        for (size_t i = 0; i < number_of_points; ++i, ++iter_x3, ++iter_y3, ++iter_z3)
        {
            double x = *iter_x3;
            double y = *iter_y3;
            double z = *iter_z3;
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
    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2Iterator<u_int8_t> iter_r(*msg, "r");
    double min_z_val = 1000, max_z_val = -1000;
    double z_values[occupancy_grid_.info.width * occupancy_grid_.info.height];
    int berm_peak = -1;
    double berm_height__right = -1000;
    for (size_t i = 0; i < occupancy_grid_.info.width * occupancy_grid_.info.height; i++)
    {
        z_values[i] = 0;
    }

    double x_min = 1000, x_max = -1000, y_min = 1000, y_max = -1000;
    for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r)
    {
        double x = *iter_x;
        double y = *iter_y;
        double z = *iter_z;
        double r = *iter_r;
        int x_idx = (int)(3 * x / occupancy_grid_.info.resolution) - 1.25 * occupancy_grid_.info.width / 2;
        int y_idx = (int)(5 * y / occupancy_grid_.info.resolution) + 1.25 * occupancy_grid_.info.height / 2;

        double z_val = -(a * x + b * y + c * z + d) / denom;
        int idx = x_idx + y_idx * occupancy_grid_.info.width;
        if (idx >= 0 && idx < (int)(occupancy_grid_.info.width * occupancy_grid_.info.height))
        {
            if (z_val < 0.1 && y>-0.5 && y<0.1)
            // if(true)
            { // if the point is on the ground and is regolith
                z_values[idx] += z_val;
                pc_density_grid_right_.data[idx]++;
                if (z_val > berm_height__right)
                {
                    berm_height__right = z_val;
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
    for (int i = 0; i < (int)(occupancy_grid_.info.width * occupancy_grid_.info.height); i++)
    {
        if (pc_density_grid_right_.data[i] > 0)
        {
            int height = (int)  1000 * (z_values[i] / pc_density_grid_right_.data[i]);
            // if (height > 0 && height < 100)
            if (height > 0)
                occupancy_grid_.data[i] = height;
            else
                occupancy_grid_.data[i] = 0;
        }
    }
    if(debug_mode_){
        printf("x_min = %f, x_max = %f, y_min = %f, y_max = %f\n", x_min, x_max, y_min, y_max);
        printf("berm height = %f\n", berm_height__right);
        printf("berm peak = %d\n", berm_peak);
    }

    filtered_occupancy_grid_.header = occupancy_grid_.header;
    filtered_occupancy_grid_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height);
    filtered_occupancy_grid_.data = occupancy_grid_.data;

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
        grow_dune(dune1_indices, dune_score, idx, filtered_occupancy_grid_.info.width, dune_count);
        dunes_indices.push_back(dune1_indices);
        dunes_scores.push_back(dune_score);
        if (dune_score > berm_score)
        {
            berm_idx = dune_count/10-1;
            berm_score = dune_score;
        }
        dune_count+=10;
        if(debug_mode_)
            printf("counter = %d, idx = %d,dune1_indices.size() = %d, dunes_score = %d\n", dune_count, idx, dune1_indices.size(), dune_score);
    }
    int berm_height__max = 0;    
    int berm_peak2 = -1;
    for(int j=0; j<dunes_indices[berm_idx].size(); j++)
    {
        if(occupancy_grid_.data[dunes_indices[berm_idx][j]] > berm_height__max)
        {
            berm_height__max = occupancy_grid_.data[dunes_indices[berm_idx][j]];
            berm_peak2 = dunes_indices[berm_idx][j];
        }
        filtered_occupancy_grid_.data[dunes_indices[berm_idx][j]] = 100;
    }

    berm_peak = berm_peak2;

    std::vector<int> dune_indices_x;
    std::vector<int> dune_indices_y;
    std::vector<int> dune_indices;
    std::vector<int> peak_indices;

    // accumulate x and y coordinates of occupancy_grid_ indices having value more than 10
    add_dune_neighbors(dune_indices_x, dune_indices_y, dune_indices, berm_peak, occupancy_grid_.info.width);

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
    if(debug_mode_){
        for (int i = best_x_start; i < best_x_end; i++)
        {
            int y = m * i + intercept;
            if (y >= 0 && y < occupancy_grid_.info.height)
                filtered_occupancy_grid_.data[i + y * occupancy_grid_.info.width] = 200;
        }
    }

    // get blob regions in filtered_occupancy_grid_ where value > 10, without using OpenCv

    berm_length_ = (best_x_end - best_x_start) * sqrt(1 + m * m) * 2;
    berm_width_ = std_dev / sqrt(1 + m * m) * 6.0;
    berm_height_ =  occupancy_grid_.data[berm_peak2]/10.0;
    if(debug_mode_){
        printf("berm_height_ = %f, berm_length_ = %f, berm_width_ = %f\n", berm_height_, berm_length_, berm_width_);
        filtered_occupancy_grid_.data[berm_peak2] = 101;

        printf("done filtering occupancy grid\n");
        publisher_og_->publish(occupancy_grid_);
        publisher_fil_->publish(filtered_occupancy_grid_);
    }

    // reset vector
    dune_indices_x.clear();
    dune_indices_y.clear();
    dune_indices.clear();
    peak_indices.clear();
    dunes_indices.clear();
    dunes_scores.clear();
}

bool BermMap::process_left(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Make occupancy grid
        if(!is_initialized_left_){
            RCLCPP_ERROR(this->get_logger(), "Error: Point cloud from left camera not initialized");
            return false;
        }
        rclcpp::Time current_time = this->now();
        int cloud_age = current_time.seconds() - (msg->header.stamp.sec + msg->header.stamp.nanosec/1e9);
        if(cloud_age > 3){
            RCLCPP_ERROR(this->get_logger(), "Error: Point cloud from left camera is %d seconds old", cloud_age);
            return false;
        }

        occupancy_grid_.header = msg->header;
        occupancy_grid_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height);
        pc_density_grid_left_.header = occupancy_grid_.header;
        pc_density_grid_left_.info = occupancy_grid_.info;
        pc_density_grid_left_.info.origin.position.z = 10;
        pc_density_grid_left_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height);

        for (int i = 0; i < (int)(occupancy_grid_.info.width * occupancy_grid_.info.height); i++)
        {
            int val = std::min(i % occupancy_grid_.info.height, occupancy_grid_.info.width - i % occupancy_grid_.info.height);
            pc_density_grid_left_.data[i] = 1;
        }

        const double number_of_points = msg->height * msg->width;

        int num_iterations = 100;
        double num_inliers = 0, max_num_inliers = 0;
        double a, b, c, d; // plane coefficients

        double max_x, max_y, max_z, min_x, min_y, min_z;
        for (size_t i = 0; i < num_iterations; i++)
        {

            sensor_msgs::PointCloud2Iterator<float> iter_x2(*msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y2(*msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z2(*msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_x3(*msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y3(*msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z3(*msg, "z");
            // randomly select 3 indices in increasing order
            int idx1 = rand() % (int)number_of_points;
            int idx2 = rand() % (int)number_of_points;
            int idx3 = rand() % (int)number_of_points;
            if (idx1 > idx2)
            {
                int temp = idx1;
                idx1 = idx2;
                idx2 = temp;
            }
            if (idx2 > idx3)
            {
                int temp = idx2;
                idx2 = idx3;
                idx3 = temp;
            }
            if (idx1 > idx2)
            {
                int temp = idx1;
                idx1 = idx2;
                idx2 = temp;
            }
            double x1 = *iter_x2;
            double y1 = *iter_y2;
            double z1 = *iter_z2;
            for (int i = idx1; i < idx2; i++)
            {
                iter_x2 += 1;
                iter_y2 += 1;
                iter_z2 += 1;
            }
            double x2 = *iter_x2;
            double y2 = *iter_y2;
            double z2 = *iter_z2;
            for (int i = idx2; i < idx3; i++)
            {
                iter_x2 += 1;
                iter_y2 += 1;
                iter_z2 += 1;
            }
            double x3 = *iter_x2;
            double y3 = *iter_y2;
            double z3 = *iter_z2;
            // get coefficients of the plane
            double a1 = y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2);
            double b1 = z1 * (x2 - x3) + z2 * (x3 - x1) + z3 * (x1 - x2);
            double c1 = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
            double d1 = -(x1 * (y2 * z3 - y3 * z2) + x2 * (y3 * z1 - y1 * z3) + x3 * (y1 * z2 - y2 * z1));

            num_inliers = 0;

            for (size_t i = 0; i < number_of_points; ++i, ++iter_x3, ++iter_y3, ++iter_z3)
            {
                double x = *iter_x3;
                double y = *iter_y3;
                double z = *iter_z3;
                double dist = fabs(a1 * x + b1 * y + c1 * z + d1) / sqrt(a1 * a1 + b1 * b1 + c1 * c1);
                if (dist < 0.01)
                {
                    num_inliers++;
                }
            }
            if (num_inliers > max_num_inliers)
            {
                max_num_inliers = num_inliers;
                a = a1;
                b = b1;
                c = c1;
                d = d1;
            }
        }
        if (c < 0)
        { // make sure the normal vector is pointing upwards
            a = -a;
            b = -b;
            c = -c;
            d = -d;
        }
        double denom = sqrt(a * a + b * b + c * c);
        // printf("Ground plane: a = %f, b = %f, c = %f, d = %f, max_inliers = %f, num_points = %f\n", a, b, c, d, max_num_inliers, number_of_points);
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<u_int8_t> iter_r(*msg, "r");
        double min_z_val = 1000, max_z_val = -1000;
        double z_values[occupancy_grid_.info.width * occupancy_grid_.info.height];
        int berm_peak = -1;
        double berm_height__left = -1000;

        for (size_t i = 0; i < occupancy_grid_.info.width * occupancy_grid_.info.height; i++)
        {
            z_values[i] = 0;
        }

        double x_min = 1000, x_max = -1000, y_min = 1000, y_max = -1000;
        for (size_t i = 0; i < number_of_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r)
        {
            double x = *iter_x;
            double y = *iter_y;
            double z = *iter_z;
            double r = *iter_r;
            int x_idx = (int)(3 * x / occupancy_grid_.info.resolution) - 0.75 * occupancy_grid_.info.width / 2;
            int y_idx = (int)(5 * y / occupancy_grid_.info.resolution) + 1.25 * occupancy_grid_.info.height / 2;

            double z_val = -(a * x + b * y + c * z + d) / denom;
            int idx = x_idx + y_idx * occupancy_grid_.info.width;
            if (idx >= 0 && idx < (int)(occupancy_grid_.info.width * occupancy_grid_.info.height))
            {
                if (r > 120.0 && z_val < 0.2 && y < 0 && y > -0.5)
                { // if the point is on the ground and is regolith
                    z_values[idx] += z_val;
                    pc_density_grid_left_.data[idx]++;
                    if (z_val > berm_height__left)
                    {
                        berm_height__left = z_val;
                        berm_peak = idx;
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
            }
        }
        for (int i = 0; i < (int)(occupancy_grid_.info.width * occupancy_grid_.info.height); i++)
        {
            if (pc_density_grid_left_.data[i] > 0)
            {
                int height = (int)1000 * (z_values[i] / pc_density_grid_left_.data[i]);
                if (height > 0)
                    occupancy_grid_.data[i] = height;
            }
        }
        // printf("x_min = %f, x_max = %f, y_min = %f, y_max = %f\n", x_min, x_max, y_min, y_max);
    }
