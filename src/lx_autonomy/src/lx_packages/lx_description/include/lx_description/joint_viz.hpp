#ifndef JOINT_VIZ_H
#define JOINT_VIZ_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointViz: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        
        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr tool_height_subscriber_;
        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();
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