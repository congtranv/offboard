#include "offboard/offboard.h"

// mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

// geometry_msgs::PoseStamped target_pose;
// bool check_reached(float, float, float);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 100, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 100, pose_cb);
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 100);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected)
	{
        ros::spinOnce();
        rate.sleep();
    }
	
	// check current state and position
	for(int i = 10; ros::ok() && i > 0; --i)
	{
		ROS_INFO_STREAM("\nCurrent state: \n" << current_state);
		ROS_INFO_STREAM("\nCurrent pose: \n" << current_pose.pose.position);	
		ros::spinOnce();
        rate.sleep();
    }

	float x,y,z;
	std::cout << "Input target: " << std::endl;
	std::cout << "x: "; std::cin >> x;
	std::cout << "y: "; std::cin >> y;
	std::cout << "z: "; std::cin >> z;  
	input(x, y, z); 

    // send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
		ROS_INFO_STREAM("\nCurrent position: \n" << current_pose.pose.position);	
	    ROS_INFO_STREAM("\nCurrent state: \n" << current_state);
		ROS_INFO_STREAM("\nTarget position: \n" << target_pose.pose.position);
        
		// publish target position
		local_pos_pub.publish(target_pose);
        
		/* 
		check_reached(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
		std::cout << check_reached << std::endl;
		if(!check_reached)
		{
			local_pos_pub.publish(target_pose);
		}
		else 
		{
			input(x+1, y, z);
		}
		*/
        ros::spinOnce();
        rate.sleep();
    }
	
	// ROS_INFO("\nwhile break\n");
    return 0;
}
