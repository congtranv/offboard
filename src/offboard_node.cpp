#include <offboard/offboard.h>

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate_(50.0);

    OffboardControl offboard;
    // offboard.initial_state(nh, rate_);
    // offboard.takeOff(rate_);
    offboard.position_control(nh, local_input_, rate_);

    return 0;
}
