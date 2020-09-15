#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include <iostream>
#include <cmath>

bool check_reached(float x, float y, float z);
void input(float x, float y, float z);

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped target_pose;

bool check_reached(float x, float y, float z)
{
	if((std::floor(x) == target_pose.pose.position.x) && (std::floor(y) ==  target_pose.pose.position.y) && (std::floor(z) == target_pose.pose.position.z))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void input(float x, float y, float z)
{
	std::cout << "Inputing target ... " << std::endl;
	// std::cout << "x: " << x; 
	target_pose.pose.position.x = x;
	// std::cout << "y: " << y; 
	target_pose.pose.position.y = y;
	// std::cout << "z: " << z; 
	target_pose.pose.position.z = z;
}
