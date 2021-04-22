#include "offboard/offboard.h"

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate_(10);

    OffboardControl offboard;
    offboard.position_control(nh, rate_);

    return 0;
}
