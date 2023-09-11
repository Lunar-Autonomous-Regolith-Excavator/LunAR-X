/* Author: Dhruv Tyagi
 * Subscribers:
 * Publishers:
 * Services:
 *
 * - Run diagnostics on robot autonomy and hardware
 * 
 * TODO
 * */

#include "lx_diagnostics/diagnostics.hpp"

Diagnostics::Diagnostics(): Node("diagnostics_node"){
    // Set up subscriptions & publishers
    setupCommunications();

    // Set up parameters from the global parameter server
    setupParams();

    RCLCPP_INFO(this->get_logger(), "Diagnostics initialized");
}

void Diagnostics::setupCommunications(){
    // Subscribers

    // Publishers

    // Clients
}

void Diagnostics::setupParams(){
    // Parameter handling
}