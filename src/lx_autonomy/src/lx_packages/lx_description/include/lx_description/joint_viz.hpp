#ifndef JOINT_VIZ_H
#define JOINT_VIZ_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lx_msgs/msg/tool_info.hpp"
#include <cmath>

class JointViz: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        const double twoPi = 2 * M_PI; // 2π
        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tool_height_subscriber_;
        rclcpp::Subscription<lx_msgs::msg::ToolInfo>::SharedPtr tool_info_subscriber_;
        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Callback function for tool_height_subscriber_
        * */
        void toolHeightCallBack(const std_msgs::msg::Float64::SharedPtr msg);

        /*
        * Callback function for tool_info_subscriber_
        * */
        void toolInfoCallBack(const lx_msgs::msg::ToolInfo::SharedPtr msg);
        // --------------------------------------

    public:
        // Functions
        /*
        * Constructor
        * */
        JointViz();

        /*
        * Destructor
        * */
        ~JointViz(){}
};

#endif