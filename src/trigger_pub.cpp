#include <ros/ros.h>
#include <std_msgs/Bool.h>

bool trigger;
std_msgs::Bool trig;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trigger_pub");
    ros::NodeHandle nh;

    ros::Publisher trigger_pub = nh.advertise<std_msgs::Bool>("/human_trigger", 100);

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
        system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
        ros::param::get("human_trig", trigger);
        if (trigger)
        {
            std::cout << "Human trigger: true\n";
        }
        else
        {
            std::cout << "Human trigger: false\n";
        }
        
        trig.data = trigger;
        trigger_pub.publish(trig);
        
        // ros::spinOnce(); 
        loop_rate.sleep();
    }
    
    return 0;
}