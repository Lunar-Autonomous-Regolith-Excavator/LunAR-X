#ifndef PARAM_SERVER_H
#define PARAM_SERVER_H

#include "rclcpp/rclcpp.hpp"


class ParamServer: public rclcpp::Node
{
    private:
        // Variables ----------------------------
        // Parameter handling
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> call_back_handle_[19];
        // --------------------------------------


        // Functions ----------------------------
        /*
        * Set up and initialize all rover-wide parameters
        * */
        void initParameters();
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