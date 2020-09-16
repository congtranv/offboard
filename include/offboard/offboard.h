#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include <iostream>
#include <cmath>

bool check_reached(void);
void input_target(void);

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped target_pose;

int target_num;
float target_pos[][3];

bool check_reached()
{
	bool reached;
	if((std::floor(current_pose.pose.position.x) == target_pose.pose.position.x) 
	     && (std::floor(current_pose.pose.position.y) ==  target_pose.pose.position.y) 
		 && (std::floor(current_pose.pose.position.z) == target_pose.pose.position.z))
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}

void input_target()
{
	std::cout << "Input target(s) position:" << std::endl;
	std::cout << "Number of target(s): "; std::cin >> target_num;
	for (int i = 0; i < target_num; i++)
	{
		std::cout << "Target (" << i << "):" <<std::endl; 
		std::cout << "x_" << i << ":"; std::cin >> target_pos[i][0];
		std::cout << "y_" << i << ":"; std::cin >> target_pos[i][1];
		std::cout << "z_" << i << ":"; std::cin >> target_pos[i][2];
	}
}
