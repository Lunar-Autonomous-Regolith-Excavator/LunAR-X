#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "rclcpp/rclcpp.hpp"
#include "lx_msgs/msg/node_diagnostics.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "lx_library/lx_utils.hpp"
#include <string>

class Diagnostics: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        std::vector<std::string> nodes_list_ {"param_server", 
                                              "external_interface", 
                                              "command_mux"};
        // Time
        unsigned int timeout_period_sec_ = 3;
        rclcpp::TimerBase::SharedPtr diagnostics_timers_[3];
        // Subscribers
        rclcpp::Subscription<lx_msgs::msg::NodeDiagnostics>::SharedPtr diagnostics_subscribers_[3];
        // Clients
        rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_params_client_;
		rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_params_client_;
        // Parameter handling
        struct lock_struct rover_soft_lock_;
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> mob_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> act_param_cb_handle_;
        // --------------------------------------

        // Functions ----------------------------
        /*
        * Set up subscribers and publishers of the node
        * */
        void setupCommunications();

        /*
        * Set up tracking of global parameters
        * */
        void setupParams();

        /*
        * Get starting values of global parameters
        * */
        void getParams();

        /*
        * Callback function for starting values of global parameters
        * */
        void paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture );

        /*
        * Callback function for diagnostics
        * */
        void diagnosticsCallBack(const lx_msgs::msg::NodeDiagnostics::SharedPtr );

        /*
        * Set rover lock if diagnostics time out
        * */
        void setRoverLock();
        // --------------------------------------

    public:
        // Functions
        /*
        * Constructor
        * */
        Diagnostics();

        /*
        * Destructor
        * */
        ~Diagnostics(){}
};

#endif