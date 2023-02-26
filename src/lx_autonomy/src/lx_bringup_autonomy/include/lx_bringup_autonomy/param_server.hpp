#ifndef PARAM_SERVER_H
#define PARAM_SERVER_H

#include "rclcpp/rclcpp.hpp"


class ParamServer: public rclcpp::Node
{
    private:
        std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_[4];
        void initParameters();
    public:
        ParamServer();
        ~ParamServer(){}

};

#endif