#ifndef STATUS_RELAY_H
#define STATUS_RELAY_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

class StatusRelay: public rclcpp::Node
{
	private:
		// Timers
		rclcpp::TimerBase::SharedPtr get_params_timer_;
		rclcpp::TimerBase::SharedPtr hw_pub_timer_;

		// Publishers
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hw_status_publisher_;
		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr op_status_publisher_;
		rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr task_status_publisher_;
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lock_status_publisher_;

		// Client
		rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_params_client_;

		/*
        * Set up reset and call timers for publisher and parameter server 
        * */
		void setupTimers();

		/*
        * Set up publisher and clients
        * */
		void setupCommunications();

        /*
        * Publish hardware status
        * */
		void pubHWStatus();

		/*
        * Publish lock status
        * */
		void pubLockStatus(bool );

		/*
        * Publish operation mode status
        * */
		void pubOpModeStatus(int );

		/*
        * Publish task mode status
        * */
		void pubTaskStatus(int );

		/*
        * Get parameters from the param server
        * */
		void getParameters();

		/*
        * Callback for shared futures from param call
        * */
		void paramCB(rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFuture );

		
	public:	
		/*
        * Constructor
        * */
		StatusRelay();

		/*
        * Destructor
        * */
		~StatusRelay(){}
};

#endif