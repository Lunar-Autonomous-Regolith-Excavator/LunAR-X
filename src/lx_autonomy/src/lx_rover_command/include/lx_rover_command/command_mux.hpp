#ifndef COMMAND_MUX_H
#define COMMAND_MUX_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "lx_msgs/msg/rover_command.hpp"
#include "lx_library/lx_utils.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"


class CommandMux: public rclcpp::Node
{
    private:
        // Variables ----------------------------
        // Time
        rclcpp::TimerBase::SharedPtr last_cmd_timer_;
        // Subscribers
        rclcpp::Subscription<lx_msgs::msg::RoverCommand>::SharedPtr rover_teleop_subscriber_;
        rclcpp::Subscription<lx_msgs::msg::RoverCommand>::SharedPtr rover_auto_subscriber_;
        std::thread teleop_cmd_pub_thread_;
        std::thread auto_cmd_pub_thread_;
        // Publishers
        rclcpp::Publisher<lx_msgs::msg::RoverCommand>::SharedPtr rover_hw_cmd_publisher_;
        // Parameter handling
        struct lock_struct rover_soft_lock_;
        OpModeEnum current_rover_op_mode_ = OpModeEnum::STANDBY;
        TaskModeEnum current_rover_task_mode_ = TaskModeEnum::IDLE;
        float max_mob_lin_vel_ = 0.5; // Reflect any default param changes here 
        float max_mob_ang_vel_ = 0.1; // Reflect any default param changes here
        float max_drum_speed_ = 0.1;  // Reflect any default param changes here
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> mob_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> act_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> op_mode_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> task_mode_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> lin_mob_vel_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> ang_mob_vel_param_cb_handle_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> drum_speed_param_cb_handle_;
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
        * Argument(s):
        *   - rover teleop message
        * 
        * Callback for /rover_teleop_cmd message
        * */
        void roverTeleopCallBack(const lx_msgs::msg::RoverCommand::SharedPtr );

        /*
        * Argument(s):
        *   - rover teleop message
        * 
        * If teleop mode enabled, pass rover teleop command from external interface to the hardware interface
        * */
        void teleopPassthrough(const lx_msgs::msg::RoverCommand::SharedPtr );

        /*
        * Argument(s):
        *   - rover autonomy message
        * 
        * Callback for /rover_auto_cmd message
        * */
        void roverAutoCallBack(const lx_msgs::msg::RoverCommand::SharedPtr );

        /*
        * Argument(s):
        *   - rover autonomy message
        * 
        * If autonomous mode enabled, pass rover autonomy command from external interface to the hardware interface
        * */
        void autoPassthrough(const lx_msgs::msg::RoverCommand::SharedPtr );

        /*
        * Argument(s):
        *   - rover teleop message
        * 
        * [Overloaded]
        * Publish command to hardware interface
        * */
        void sendCmdToHardware(const lx_msgs::msg::RoverCommand::SharedPtr );
        void sendCmdToHardware(geometry_msgs::msg::Twist& , float& ,float& );
        // --------------------------------------

    public:
        // Functions
        /*
        * Constructor
        * */
        CommandMux();

        /*
        * Destructor
        * */
        ~CommandMux(){}

};

#endif