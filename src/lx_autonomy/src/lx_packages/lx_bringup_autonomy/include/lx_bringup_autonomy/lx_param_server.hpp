#ifndef PARAM_SERVER_H
#define PARAM_SERVER_H

#include "rclcpp/rclcpp.hpp"
#include "lx_msgs/msg/node_diagnostics.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"


class ParamServer: public rclcpp::Node
{
    private:
        // Variables ----------------------------
        unsigned int diagnostic_pub_period_ = 1;
        bool rover_lock_mobility_ = true;
        bool rover_lock_actuation_ = true;
        int rover_op_mode_ = 0;
        int rover_task_mode_ = 0;
        // Timer
        rclcpp::TimerBase::SharedPtr diagnostic_pub_timer_;
        // Publisher
        rclcpp::Publisher<lx_msgs::msg::NodeDiagnostics>::SharedPtr diagnostic_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr modes_marker_publisher_;
        // Parameter handling
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> call_back_handle_[18];
        // --------------------------------------


        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Set up and initialize all rover-wide parameters
        * */
        void initParameters();
        
        /*
        * Diagnostic heartbeat published at a fixed rate
        * */
        void diagnosticPublish();

        /*
        * Display operation mode, task mode and lock status on Rviz
        * */
        void updateModeMarker();
        // --------------------------------------

    public:
        // Functions
        /*
        * Constructor
        * */
        ParamServer();
        
        /*
        * Destructor
        * */
        ~ParamServer(){}
};

#endif