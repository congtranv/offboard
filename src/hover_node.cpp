#include <iostream>
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose, target_pose;

void state_cb_(const mavros_msgs::State::ConstPtr& msg);
void localPose_cb_(const geometry_msgs::PoseStamped::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hover");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb_);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, localPose_cb_);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Rate rate(10.0);

    while(ros::ok() && !current_state.connected)
	{
        std::cout << "[ INFO] Waiting for FCU connection...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] FCU connected \n";

    for(int i = 50; ros::ok() && i > 0; --i)
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", 
                current_pose.pose.position.x, 
                current_pose.pose.position.y, 
                current_pose.pose.position.z);

    // input target position, hovering at current (x, y) 
    target_pose.pose.position.x = current_pose.pose.position.x;
    target_pose.pose.position.y = current_pose.pose.position.y;
	std::cout << "\nInput z for hovering (m): " << std::endl;
	std::cout << "z =  "; std::cin >> target_pose.pose.position.z;

    // send a few setpoints before starting
    std::cout << "[ INFO] Setting OFFBOARD stream...\n";
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Set OFFBOARD stream done \n";

    std::cout << "[ INFO] Waiting arm and takeoff... \n";
    while (ros::ok() && !current_state.armed && (current_state.mode != "OFFBOARD"))
    {
        ros::spinOnce();
        rate.sleep();
    }

    // publish target, keep drone hovering
    while(ros::ok())
    {
        target_pose.header.stamp = ros::Time::now();   
        local_pos_pub.publish(target_pose);

        if (!current_state.armed && (current_state.mode != "OFFBOARD"))
        {
            break;
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}

void state_cb_(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void localPose_cb_(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}