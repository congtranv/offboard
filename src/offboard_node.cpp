#include "offboard/offboard.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // if true run offboard control else initialize parameters and receive messages
    bool input_setpoint = true;

    OffboardControl *offboard = new OffboardControl(nh, nh_private, input_setpoint);
    ros::spin();
        
    return 0;
}

