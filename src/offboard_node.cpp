#include <offboard/offboard.h>

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    OffboardControl offboard;
    offboard.position_pub(nh);

    return 0;
}
