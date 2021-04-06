#include <offboard/offboard.h>

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    OffboardControl offboard;
    offboard.initial_state(nh, rate);
    offboard.takeOff(rate);
    offboard.position_control(local_input_, rate);

    return 0;
}
