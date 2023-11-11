#ifndef BAYES_FILTER_H
#define BAYES_FILTER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "lx_msgs/srv/switch.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <thread>


class BayesFilter
{
    private:
        const double SIGMA_T = 0.1;
        const double LOCALIZATION_VARIANCE = 20;
        float cellElevation;
        float cellVariance;

    public:
        // Functions
        void updateCell(int z, float sigma_t);
        float getCellElevation();
        float getCellVariance();

        // Constructor
        BayesFilter();

};  

#endif