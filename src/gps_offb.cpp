#include "offboard/offboard.h"

int main(int argc, char **argv) 
{

    // initialize ros node
    ros::init(argc, argv, "gps_offb");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> 
            ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix> 
            ("mavros/global_position/global", 10, globalPosition_cb);
    ros::Subscriber batt_sub = nh.subscribe<sensor_msgs::BatteryState> 
            ("mavros/battery", 10, battery_cb);

    // publisher
    ros::Publisher goal_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped> 
            ("mavros/setpoint_position/global", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for fcu connection
    while (ros::ok() && !current_state.connected) 
    {
        ROS_INFO_ONCE("Waiting for FCU connection ...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    // wait for position information
    while (ros::ok() && !global_position_received) 
    {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GPS position received");

    // check battery status
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery: %.2f \n", batt_percent);
        ros::spinOnce();
        rate.sleep();
    }

    // set target position
    input_global_target();
    goal_position.pose.position.latitude = goal_pos[0][0];
    goal_position.pose.position.longitude = goal_pos[0][1];
    goal_position.pose.position.altitude = goal_pos[0][2];

    // send a few setpoints before starting
    for (int i=100; ros::ok() && i>0; --i) 
    {
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Ready");

    int i=0;
    while (ros::ok()) 
    {
        if (i < goal_num)
        {
            goal_position.pose.position.latitude = goal_pos[i][0];
            goal_position.pose.position.longitude = goal_pos[i][1];
            goal_position.pose.position.altitude = goal_pos[i][2];
            goal_position.header.stamp = ros::Time::now();
            goal_pos_pub.publish(goal_position);

            ros::spinOnce();
        	rate.sleep();
        }
        else
        {
            goal_position.pose.position.latitude = goal_pos[goal_num-1][0];
            goal_position.pose.position.longitude = goal_pos[goal_num-1][1];
            goal_position.pose.position.altitude = goal_pos[goal_num-1][2];
            goal_position.header.stamp = ros::Time::now();
            goal_pos_pub.publish(goal_position);

            ros::spinOnce();
        	rate.sleep();
        }

        // check when drone reached goal      
		bool check = check_goal();
		std::cout << check << std::endl;
		if(check)
		{
			i = i + 1;
			ros::spinOnce();
		    rate.sleep();
		}
		else 
		{
			continue;
			ros::spinOnce();
		    rate.sleep();
		}

        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery: %.2f \n", batt_percent);
        distance = measureGPS(global_position.latitude, global_position.longitude, global_position.altitude, 
            goal_position.pose.position.latitude, goal_position.pose.position.longitude, goal_position.pose.position.altitude);
        std::printf("Distance to goal: %.2f m \n", distance);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
