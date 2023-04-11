#include <cstdio>
#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>


using std::placeholders::_1;
using namespace std::chrono_literals;

class BermMap : public rclcpp::Node
{
public:
    BermMap()
        : Node("lx_berm")
    {
        subscription_right = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "camera2/depth/color/points", 100, std::bind(&BermMap::topic_callback_right, this, _1));
        // subscription_left = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "camera1/depth/color/points", 100, std::bind(&BermMap::topic_callback_left, this, _1));

        publisher_og = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_berm/occupancy_grid2", 1);
        publisher_fil = this->create_publisher<nav_msgs::msg::OccupancyGrid>("lx_berm/occupancy_grid_filtered", 1);
        publisher_berm_eval = this->create_publisher<std_msgs::msg::Float32>("lx_berm/berm_eval", 1);
        setTargetBermHeight(0.15);
        occupancy_grid.info.resolution = 0.05;
        occupancy_grid.info.width = 200;
        occupancy_grid.info.height = 200;
        occupancy_grid.info.origin.position.x = -5;
        occupancy_grid.info.origin.position.y = -5;
        occupancy_grid.info.origin.position.z = 0;
        occupancy_grid.info.origin.orientation.x = 0;
        occupancy_grid.info.origin.orientation.y = 0;
        occupancy_grid.info.origin.orientation.z = 0;
        occupancy_grid.info.origin.orientation.w = 1;
        filtered_occupancy_grid = occupancy_grid;
    }

    void setTargetBermHeight(double height = 0.15)
    {
        berm_target_height = height;
    }

private:
    void add_dune_neighbors(std::vector<int> &dune_x, std::vector<int> &dune_y, std::vector<int> &dune_indices, int idx, int width)
    {
        int x_idx = idx % width;
        int y_idx = idx / width;

        for (int i = x_idx - 1; i < x_idx + 2; i++)
        {
            for (int j = y_idx - 1; j < y_idx + 2; j++)
            {
                int candidate_idx = i + j * width;
                if (!(i == x_idx && j == y_idx) && occupancy_grid.data[candidate_idx] > 20)
                {
                    // check if i does not exist in dune_indices_x
                    if (std::find(dune_indices.begin(), dune_indices.end(), candidate_idx) == dune_indices.end())
                    {
                        dune_x.push_back(i);
                        dune_y.push_back(j);
                        dune_indices.push_back(candidate_idx);
                        // score += occupancy_grid.data[candidate_idx]*occupancy_grid.data[candidate_idx];
                        add_dune_neighbors(dune_x, dune_y, dune_indices, candidate_idx, width);
                    }
                }
            }
        }
    }

    void grow_dune(std::vector<int> &dune_indices,int &score, int idx, int width, int dune_counter)
    {
        // printf("grow_dune %d: %d", dune_counter, idx);
        int x_idx = idx % width;
        int y_idx = idx / width;
        filtered_occupancy_grid.data[idx] = 0;

        for (int i = x_idx - 1; i < x_idx + 2; i++)
        {
            for (int j = y_idx - 1; j < y_idx + 2; j++)
            {
                int candidate_idx = i + j * width;
                if (!(i == x_idx && j == y_idx) && occupancy_grid.data[candidate_idx] > 20)
                {
                    // check if i does not exist in dune_indices_x
                    if (std::find(dune_indices.begin(), dune_indices.end(), candidate_idx) == dune_indices.end())
                    {
                        dune_indices.push_back(candidate_idx);
                        score += occupancy_grid.data[candidate_idx]*occupancy_grid.data[candidate_idx];
                        grow_dune(dune_indices, score, candidate_idx, width, dune_counter);
                    }
                }
            }
        }
    }

    void topic_callback_right(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Make occupancy grid

        occupancy_grid.header = msg->header;
        occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height);

        pc_density_grid_right.header = occupancy_grid.header;
        pc_density_grid_right.info = occupancy_grid.info;
        pc_density_grid_right.info.origin.position.z = 10;
        pc_density_grid_right.data.resize(occupancy_grid.info.width * occupancy_grid.info.height);

        for (int i = 0; i < (int)(occupancy_grid.info.width * occupancy_grid.info.height); i++)
        {
            int val = std::min(i % occupancy_grid.info.height, occupancy_grid.info.width - i % occupancy_grid.info.height);
            pc_density_grid_right.data[i] = 1;
        }

        const double number_of_points = msg->height * msg->width;

        // fit a ground plane to the point cloud using RANSAC
        // 1. randomly select 4 points
        // 2. fit a plane to the 4 points
        // 3. count the number of points that are within a threshold distance from the plane
        // 4. repeat 1-3 for a large number of iterations

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

            // printf("candidate coeffs: %f %f %f %f\n ",a1,b1,c1,d1);
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
        // printf("Ground plane: a = %f, b = %f, c = %f, d = %f, max_inliers = %f, num_points = %f\n", a, b, c, d, max_num_inliers, number_of_points);
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<u_int8_t> iter_r(*msg, "r");
        double min_z_val = 1000, max_z_val = -1000;
        double z_values[occupancy_grid.info.width * occupancy_grid.info.height];
        int berm_peak = -1;
        berm_height_right = -1000;
        for (size_t i = 0; i < occupancy_grid.info.width * occupancy_grid.info.height; i++)
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
            int x_idx = (int)(3 * x / occupancy_grid.info.resolution) - 1.25 * occupancy_grid.info.width / 2;
            int y_idx = (int)(5 * y / occupancy_grid.info.resolution) + 1.25 * occupancy_grid.info.height / 2;

            double z_val = -(a * x + b * y + c * z + d) / denom;
            int idx = x_idx + y_idx * occupancy_grid.info.width;
            if (idx >= 0 && idx < (int)(occupancy_grid.info.width * occupancy_grid.info.height))
            {
                if (z_val < 0.1 && y>-0.5 && y<0.1)
                // if(true)
                { // if the point is on the ground and is regolith
                    z_values[idx] += z_val;
                    pc_density_grid_right.data[idx]++;
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
        // berm_height_right -= min_z_val/3;   
        for (int i = 0; i < (int)(occupancy_grid.info.width * occupancy_grid.info.height); i++)
        {
            if (pc_density_grid_right.data[i] > 0)
            {
                // int height = (int)fabs(1000*(z_values[i]/pc_density_grid_right.data[i]));
                int height = (int)  1000 * (z_values[i] / pc_density_grid_right.data[i]);
                // if (height > 0 && height < 100)
                if (height > 0)
                    occupancy_grid.data[i] = height;
                else
                    occupancy_grid.data[i] = 0;
            }
        }

        // printf("x_min = %f, x_max = %f, y_min = %f, y_max = %f\n", x_min, x_max, y_min, y_max);
        // printf("berm height = %f\n", berm_height_right);
        // printf("berm peak = %d\n", berm_peak);

        filtered_occupancy_grid.header = occupancy_grid.header;
        filtered_occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height);
        filtered_occupancy_grid.data = occupancy_grid.data;
        for (int i = 0; i < (int)(occupancy_grid.info.width * occupancy_grid.info.height); i++)
        {
            int x_idx = i % occupancy_grid.info.width;
            int y_idx = i / occupancy_grid.info.width;
            int num_neighbors = 0;
            int sum = 0;
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    int idx = (x_idx + x) + (y_idx + y) * occupancy_grid.info.width;
                    if (idx >= 0 && idx < (int)(occupancy_grid.info.width * occupancy_grid.info.height))
                    {
                        if (occupancy_grid.data[idx] > 0)
                        {
                            sum += occupancy_grid.data[idx];
                            num_neighbors++;
                        }
                    }
                }
            }
            // printf("num_neighbors = %d\n", num_neighbors);
            if (num_neighbors > 0)
            {
                filtered_occupancy_grid.data[i] = sum / num_neighbors;
            }
        }
        // apply sobel filter to filtered_occupancy_grid
        int sobel_x[9] = {-1, 0, 1, -2, 0, 2, -1, 0, 1};
        // int sobel_x[9] = {-1, 2, -1, -1, 2, -1, -1, 2, -1};
        int sobel_y[9] = {-1, -2, -1, 0, 0, 0, 1, 2, 1};
        // int sobel_y[9] = {-1, -1, -1, 2,2,2, -1, -1, -1};

        int peak_idx = -1;
        int peak_val = -1;

        for (int i = 0; i < (int)(occupancy_grid.info.width * occupancy_grid.info.height); i++)
        {
            int x_idx = i % occupancy_grid.info.width;
            int y_idx = i / occupancy_grid.info.width;
            int num_neighbors = 0;
            int sum = 0;
            int sobel_x_sum = 0;
            int sobel_y_sum = 0;
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    int idx = (x_idx + x) + (y_idx + y) * occupancy_grid.info.width;
                    if (idx >= 0 && idx < (int)(occupancy_grid.info.width * occupancy_grid.info.height))
                    {
                        if (filtered_occupancy_grid.data[idx] > 0)
                        {
                            sobel_x_sum += filtered_occupancy_grid.data[idx] * sobel_x[num_neighbors];
                            sobel_y_sum += filtered_occupancy_grid.data[idx] * sobel_y[num_neighbors] + 10;
                            num_neighbors++;
                            // printf("sobel_x_sum = %d, sobel_y_sum = %d\n", sobel_x_sum, sobel_y_sum);
                        }
                    }
                }
            }
            // printf("num_neighbors = %d\n", num_neighbors);
            if (num_neighbors > 0)
            {
                // int val = sqrt(sobel_x_sum*sobel_x_sum + sobel_y_sum*sobel_y_sum)/5;
                // int val = std::abs(sobel_y_sum*sobel_x_sum/100);
                int val = 3 * sobel_x_sum / 4;

                // filtered_occupancy_grid.data[i] = (int)fabs(sobel_y_sum);
                if (occupancy_grid.data[i] > 20)
                {
                    filtered_occupancy_grid.data[i] = occupancy_grid.data[i];
                    // printf("%d %d,",i, val);
                }
                else
                    filtered_occupancy_grid.data[i] = 0;
                if (val > peak_val)
                {
                    peak_val = val;
                    peak_idx = i;
                }
            }
        }


        std::vector<std::vector<int>> dunes_indices;
        std::vector<int> dunes_scores;
        int idx = 0;
        int dune_count = 10;
        int berm_idx = -1, berm_score = 0;
        while(idx!=filtered_occupancy_grid.data.size())
        {
            // set idx to the next index of occupancy_grid.data having value > 10
            idx = std::find_if(filtered_occupancy_grid.data.begin()+idx+1, filtered_occupancy_grid.data.end(), [](int i){return i>10;}) - filtered_occupancy_grid.data.begin();
            std::vector<int> dune1_indices;
            int dune_score = 0;
            grow_dune(dune1_indices, dune_score, idx, filtered_occupancy_grid.info.width, dune_count);
            dunes_indices.push_back(dune1_indices);
            dunes_scores.push_back(dune_score);
            if (dune_score > berm_score)
            {
                berm_idx = dune_count/10-1;
                berm_score = dune_score;
            }
            dune_count+=10;
            // printf("counter = %d, idx = %d,dune1_indices.size() = %d, dunes_score = %d\n", dune_count, idx, dune1_indices.size(), dune_score);
        }
        int berm_height_max = 0;    
        int berm_peak2 = -1;
        for(int j=0; j<dunes_indices[berm_idx].size(); j++)
        {
            if(occupancy_grid.data[dunes_indices[berm_idx][j]] > berm_height_max)
            {
                berm_height_max = occupancy_grid.data[dunes_indices[berm_idx][j]];
                berm_peak2 = dunes_indices[berm_idx][j];
            }
            filtered_occupancy_grid.data[dunes_indices[berm_idx][j]] = 100;
        }

        berm_peak = berm_peak2;
        // make an int vector to store indices of dunes
        std::vector<int> dune_indices_x;
        std::vector<int> dune_indices_y;
        std::vector<int> dune_indices;
        std::vector<int> peak_indices;

        // accumulate x and y coordinates of occupancy_grid indices having value more than 10
        add_dune_neighbors(dune_indices_x, dune_indices_y, dune_indices, berm_peak, occupancy_grid.info.width);

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
            // printf("m = %f, c = %f\n", m, intercept);
            double mean_x = sum_x / n;
            double mean_y = sum_y / n;
            double sum_err = 0;
            for (int i = 0; i < n; i++)
            {
                double err = y[i] - (m * x[i] + intercept);
                sum_err += err * err;
            }
            std_dev = sqrt(sum_err / n);
            // printf("std_dev = %f", std_dev);
        }

        int x_start = -1;
        int x_end = -1;
        int best_x_start = 0;
        int best_x_end = 0;
        bool isCounting = false;
        for (int i = 0; i < occupancy_grid.info.width; i++)
        {
            int y = m * i + intercept;
            // printf ("y = %d, x = %d, height = %d\n", y, i, occupancy_grid.data[i + y * occupancy_grid.info.width]);
            if (y >= 0 && y < occupancy_grid.info.height && occupancy_grid.data[i + y * occupancy_grid.info.width] > 20)

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
            if (y >= 0 && y < occupancy_grid.info.height)
            {
                // occupancy_grid.data[i + y * occupancy_grid.info.width] = 100;
                filtered_occupancy_grid.data[i + y * occupancy_grid.info.width] = 200;
            }
        }
        // occupancy_grid.data[berm_peak] = 200;

        // get blob regions in filtered_occupancy_grid where value > 10, without using OpenCv

        double dune_length = (best_x_end - best_x_start) * sqrt(1 + m * m) * 2;
        double dune_width = std_dev / sqrt(1 + m * m) * 6.0;
        double dune_height =  occupancy_grid.data[berm_peak2]/10.0;
        printf("berm_height = %f, berm_length = %f, berm_width = %f\n", dune_height, dune_length, dune_width);
        filtered_occupancy_grid.data[berm_peak2] = 101;
        // filtered_occupancy_grid.data[peak_idx] = 100;

        // printf("done filtering occupancy grid\n");
        publisher_og->publish(occupancy_grid);
        publisher_fil->publish(filtered_occupancy_grid);
        berm_height_msg.data = (berm_height_left + berm_height_right) / 2;
        publisher_berm_eval->publish(berm_height_msg);

        // reset vector
        dune_indices_x.clear();
        dune_indices_y.clear();
        dune_indices.clear();
        peak_indices.clear();
        dunes_indices.clear();
        dunes_scores.clear();
    }

    void topic_callback_left(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Make occupancy grid

        occupancy_grid.header = msg->header;
        occupancy_grid.data.resize(occupancy_grid.info.width * occupancy_grid.info.height);

        pc_density_grid_left.header = occupancy_grid.header;
        pc_density_grid_left.info = occupancy_grid.info;
        pc_density_grid_left.info.origin.position.z = 10;
        pc_density_grid_left.data.resize(occupancy_grid.info.width * occupancy_grid.info.height);

        for (int i = 0; i < (int)(occupancy_grid.info.width * occupancy_grid.info.height); i++)
        {
            int val = std::min(i % occupancy_grid.info.height, occupancy_grid.info.width - i % occupancy_grid.info.height);
            pc_density_grid_left.data[i] = 1;
        }

        const double number_of_points = msg->height * msg->width;

        // fit a ground plane to the point cloud using RANSAC
        // 1. randomly select 4 points
        // 2. fit a plane to the 4 points
        // 3. count the number of points that are within a threshold distance from the plane
        // 4. repeat 1-3 for a large number of iterations

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

            // printf("candidate coeffs: %f %f %f %f\n ",a1,b1,c1,d1);
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
        double z_values[occupancy_grid.info.width * occupancy_grid.info.height];
        int berm_peak = -1;
        berm_height_left = -1000;

        for (size_t i = 0; i < occupancy_grid.info.width * occupancy_grid.info.height; i++)
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
            int x_idx = (int)(3 * x / occupancy_grid.info.resolution) - 0.75 * occupancy_grid.info.width / 2;
            int y_idx = (int)(5 * y / occupancy_grid.info.resolution) + 1.25 * occupancy_grid.info.height / 2;

            double z_val = -(a * x + b * y + c * z + d) / denom;
            int idx = x_idx + y_idx * occupancy_grid.info.width;
            if (idx >= 0 && idx < (int)(occupancy_grid.info.width * occupancy_grid.info.height))
            {
                if (r > 120.0 && z_val < 0.2 && y < 0 && y > -0.5)
                { // if the point is on the ground and is regolith
                    z_values[idx] += z_val;
                    pc_density_grid_left.data[idx]++;
                    if (z_val > berm_height_left)
                    {
                        berm_height_left = z_val;
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
        // berm_height_left -= min_z_val/3;
        for (int i = 0; i < (int)(occupancy_grid.info.width * occupancy_grid.info.height); i++)
        {
            if (pc_density_grid_left.data[i] > 0)
            {
                // int height = (int)fabs(1000*(z_values[i]/pc_density_grid_left.data[i]));
                int height = (int)1000 * (z_values[i] / pc_density_grid_left.data[i]);
                if (height > 0)
                    occupancy_grid.data[i] = height;
            }
        }
        // occupancy_grid.data[berm_peak] = 200;
        // printf("x_min = %f, x_max = %f, y_min = %f, y_max = %f\n", x_min, x_max, y_min, y_max);
        printf("min_z_val = %f, max_z_val = %f\n", min_z_val, max_z_val);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_right, subscription_left;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_og, publisher_fil, publisher_pc_density_right, publisher_pc_density_left;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_berm_eval;
    double berm_target_height;
    double berm_height_left, berm_height_right;

    nav_msgs::msg::OccupancyGrid occupancy_grid, filtered_occupancy_grid;
    nav_msgs::msg::OccupancyGrid pc_density_grid_right, pc_density_grid_left;
    std_msgs::msg::Float32 berm_height_msg;
    rclcpp::TimerBase::SharedPtr timer_;
};