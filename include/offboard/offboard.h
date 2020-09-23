#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>

#include <iostream>
#include <cmath>

bool check_position(void);
bool check_orientation(void);
void input_target(void);

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped target_pose;

int target_num;
// float target_pos[10][3];
float target_pos[10][7];

bool check_position()
{
	bool reached;
	if(((target_pose.pose.position.x - 0.1) < current_pose.pose.position.x)
	 && (current_pose.pose.position.x < (target_pose.pose.position.x + 0.1)) 
	 && ((target_pose.pose.position.y - 0.1) < current_pose.pose.position.y)
	 && (current_pose.pose.position.y < (target_pose.pose.position.y + 0.1))
	 && ((target_pose.pose.position.z - 0.1) < current_pose.pose.position.z)
	 && (current_pose.pose.position.z < (target_pose.pose.position.z + 0.1)))
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}


bool check_orientation()
{
	bool reached;
		
	if(((target_pose.pose.orientation.x - 0.1) < current_pose.pose.orientation.x)
	 && (current_pose.pose.orientation.x < (target_pose.pose.orientation.x + 0.1)) 
	 && ((target_pose.pose.orientation.y - 0.1) < current_pose.pose.orientation.y)
	 && (current_pose.pose.orientation.y < (target_pose.pose.orientation.y + 0.1))
	 && ((target_pose.pose.orientation.z - 0.1) < current_pose.pose.orientation.z)
	 && (current_pose.pose.orientation.z < (target_pose.pose.orientation.z + 0.1))
	 && ((target_pose.pose.orientation.w - 0.1) < current_pose.pose.orientation.w)
	 && (current_pose.pose.orientation.w < (target_pose.pose.orientation.w + 0.1)))
	
	/* if((current_pose.pose.orientation.x == target_pose.pose.orientation.x)&&(current_pose.pose.orientation.y == target_pose.pose.orientation.y)&&(current_pose.pose.orientation.z == target_pose.pose.orientation.z)&&(current_pose.pose.orientation.w == target_pose.pose.orientation.w)) */
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
		std::cout << "Target (" << i+1 << ") position:" <<std::endl; 
		std::cout << "pos_x_" << i+1 << ":"; std::cin >> target_pos[i][0];
		std::cout << "pos_y_" << i+1 << ":"; std::cin >> target_pos[i][1];
		std::cout << "pos_z_" << i+1 << ":"; std::cin >> target_pos[i][2];
		
		std::cout << "Target (" << i+1 << ") orientation:" <<std::endl; 
		std::cout << "ort_x_" << i+1 << ":"; std::cin >> target_pos[i][3];
		std::cout << "ort_y_" << i+1 << ":"; std::cin >> target_pos[i][4];
		std::cout << "ort_z_" << i+1 << ":"; std::cin >> target_pos[i][5];
		std::cout << "ort_w_" << i+1 << ":"; std::cin >> target_pos[i][6];
	}
}
