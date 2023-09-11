#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include "rclcpp/rclcpp.hpp"

class Diagnostics: public rclcpp::Node
{
    private:
        // Variables & pointers -----------------
        // Time
        
        // Subscribers

        // Clients

        // Publishers

        // Parameter handling

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