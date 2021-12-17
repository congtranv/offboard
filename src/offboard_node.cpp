#include"offboard/offboard.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    OffboardControl *offboard = new OffboardControl(nh, nh_private);
    // OffboardControl offboard(nh, nh_private);
    ros::spin();
        
    return 0;
}