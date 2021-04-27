#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <cstdlib>

std::vector<double> x, y, z;
geometry_msgs::PoseStamped target, current;
std_msgs::Float64 check;
double max, min, range;

inline void local_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_pub");
    ros::NodeHandle nh;

    ros::Publisher target_pub = nh.advertise<geometry_msgs::PoseStamped>("/target_position", 100);
    ros::Publisher error_pub = nh.advertise<std_msgs::Float64>("/check_error_pos", 100);
    ros::Subscriber local_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 100, local_cb);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        min = 4.5; max = 5.5;
        range = (max-min) * ((((double) rand()) / (double) RAND_MAX)) + min ;
        
        target.pose.position.x = range;
        target.pose.position.y = 0;
        target.pose.position.z = 0;
        check.data = sqrt((5 - current.pose.position.x)*(5 - current.pose.position.x)
        + (0 - current.pose.position.y)*(0 - current.pose.position.y));
        std::printf("\ntarget: [%.3f, %.3f, %.3f]\n", target.pose.position.x, target.pose.position.y, target.pose.position.z);
        std::cout << "error: " << check.data << "\n";
        target_pub.publish(target);
        error_pub.publish(check);
        // if (check.data < 0.2)
        // {
        //     std::printf("\nerror < 0.2\n");
        //     break;
        // }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}