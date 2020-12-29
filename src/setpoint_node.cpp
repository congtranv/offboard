#include "offboard/conversion.h"

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "setpoint");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix> 
            ("mavros/global_position/global", 10, globalPosition_cb);
    ros::Subscriber batt_sub = nh.subscribe<sensor_msgs::BatteryState> 
            ("mavros/battery", 10, battery_cb);

    // service
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

    // publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected)
	{
        std::cout << "[ INFO] Waiting for FCU connection...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] FCU connected \n";

    // wait for GPS information
    while (ros::ok() && !global_position_received) 
    {
        std::cout << "[ INFO] Waiting for GPS signal...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] GPS position received \n";
    ros::Duration(1).sleep();
    std::cout << "[ INFO] Checking status...\n";
    ros::Duration(2).sleep();

	// check current pose
	for(int i = 100; ros::ok() && i > 0; --i)
	{
        std::printf("Current local position: [%.3f, %.3f, %.3f]\n", 
                     current_pose.pose.position.x, 
                     current_pose.pose.position.y, 
                     current_pose.pose.position.z);
		
        std::printf("Current GPS position: [%f, %f, %.3f]\n", 
                     global_position.latitude, 
                     global_position.longitude, 
                     global_position.altitude);

        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery capacity: %.1f \n", batt_percent);
        std::cout << "\n";
		ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Check status done \n";
        // init reference point
        refpoint.latitude = global_position.latitude;
        refpoint.longitude = global_position.longitude;
        refpoint.altitude = global_position.altitude;
        std::printf("Reference GPS position: [%f, %f, %.3f]\n", 
                     refpoint.latitude, 
                     refpoint.longitude, 
                     refpoint.altitude);
    ros::Duration(1).sleep();

    // set target pose
    input_target();
    if (input_type == true) // local setpoint
    {
        target_pose.pose.position.x = target_pos[0][0];
        target_pose.pose.position.y = target_pos[0][1];
        target_pose.pose.position.z = target_pos[0][2];
        roll  = radian(target_pos[0][3]);
        pitch = radian(target_pos[0][4]);
        yaw   = radian(target_pos[0][5]);
        q.setRPY(roll, pitch, yaw);
	    tf::quaternionTFToMsg(q, target_pose.pose.orientation);
    }
    else // global setpoint
    {
        enu_goal = WGS84ToENU(goal_pos[0][0], goal_pos[0][1], goal_pos[0][2],
                    refpoint.latitude, refpoint.longitude, refpoint.altitude);
        target_pose.pose.position.x = enu_goal.x;
        target_pose.pose.position.y = enu_goal.y;
        target_pose.pose.position.z = enu_goal.z;
        roll  = radian(0);
        pitch = radian(0);
        yaw   = radian(0);
        q.setRPY(roll, pitch, yaw);
	    tf::quaternionTFToMsg(q, target_pose.pose.orientation);
    }

    std::cout << "[ INFO] Setting OFFBOARD stream...\n";
    ros::Duration(1).sleep();
    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Set OFFBOARD stream done \n";
    std::cout << "[ INFO] Ready to switch ARM and OFFBOARD \n";
    ros::Duration(2).sleep();

    int i = 0;
    while (ros::ok() && (input_type ==true))
    {
        std::printf("Current local position: [%.3f, %.3f, %.3f]\n", 
                     current_pose.pose.position.x, 
                     current_pose.pose.position.y, 
                     current_pose.pose.position.z);
        std::printf("Next local position: [%.3f, %.3f, %.3f]\n", 
                            target_pos[i][0], 
                            target_pos[i][1],
                            target_pos[i][2]);

        if (i < (target_num -1))
        {
            final_check = false;
            target_pose.pose.position.x = target_pos[i][0];
            target_pose.pose.position.y = target_pos[i][1];
            target_pose.pose.position.z = target_pos[i][2];
            roll  = radian(target_pos[i][3]);
            pitch = radian(target_pos[i][4]);
            yaw   = radian(target_pos[i][5]);
            q.setRPY(roll, pitch, yaw);
	        tf::quaternionTFToMsg(q, target_pose.pose.orientation);

            target_pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(target_pose);
    		ros::spinOnce();
            rate.sleep();
        }
        else
        {
            final_check = true;
            target_pose.pose.position.x = target_pos[target_num - 1][0];
            target_pose.pose.position.y = target_pos[target_num - 1][1];
            target_pose.pose.position.z = target_pos[target_num - 1][2];
            roll  = radian(target_pos[target_num - 1][3]);
            pitch = radian(target_pos[target_num - 1][4]);
            yaw   = radian(target_pos[target_num - 1][5]);
            q.setRPY(roll, pitch, yaw);
	        tf::quaternionTFToMsg(q, target_pose.pose.orientation);

            target_pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(target_pose);
    		ros::spinOnce();
            rate.sleep();
        }
        bool check = check_position(target_pose.pose.position.x,
                                    target_pose.pose.position.y,
                                    target_pose.pose.position.z,
                                    current_pose.pose.position.x,
                                    current_pose.pose.position.y,
                                    current_pose.pose.position.z);
        std::cout << check << std::endl;
		if(check && !final_check)
		{
            std::printf("[ INFO] Reached position: [%.3f, %.3f, %.3f]\n", 
                            current_pose.pose.position.x, 
                            current_pose.pose.position.y, 
                            current_pose.pose.position.z);
    		        
            std::printf("Next local position: [%.3f, %.3f, %.3f]\n", 
                            target_pos[i+1][0], 
                            target_pos[i+1][1],
                            target_pos[i+1][2]);
            wgs84_curr = ENUToWGS84(current_pose.pose.position.x, 
                                    current_pose.pose.position.y,
                                    current_pose.pose.position.z,
                                    refpoint.latitude,
                                    refpoint.longitude,
                                    refpoint.altitude);
            files(current_pose.pose.position.x,
                  current_pose.pose.position.y,
                  current_pose.pose.position.z,
                  wgs84_curr.latitude,
                  wgs84_curr.longitude,
                  wgs84_curr.altitude);
            files(0, 0, 0,
                  global_position.latitude,
                  global_position.longitude,
                  global_position.altitude);
            ros::Duration(5).sleep();
			i = i + 1;
			ros::spinOnce();
		    rate.sleep();
		}
        else if (check && final_check)
        {
            std::printf("[ INFO] Reached FINAL position: [%.3f, %.3f, %.3f]\n", 
                            current_pose.pose.position.x, 
                            current_pose.pose.position.y, 
                            current_pose.pose.position.z);
    		        
            std::printf("[ INFO] Ready to LANDING \n");
            wgs84_curr = ENUToWGS84(current_pose.pose.position.x, 
                                    current_pose.pose.position.y,
                                    current_pose.pose.position.z,
                                    refpoint.latitude,
                                    refpoint.longitude,
                                    refpoint.altitude);
            files(current_pose.pose.position.x,
                  current_pose.pose.position.y,
                  current_pose.pose.position.z,
                  wgs84_curr.latitude,
                  wgs84_curr.longitude,
                  wgs84_curr.altitude);
            files(0, 0, 0,
                  global_position.latitude,
                  global_position.longitude,
                  global_position.altitude);
            ros::Duration(5).sleep();

			set_mode.request.custom_mode = "AUTO.LAND";
    	    if( set_mode_client.call(set_mode) && set_mode.response.mode_sent)
            {
    		    std::cout << "[ INFO] AUTO.LAND enabled \n";
                break;
            }
        }
		else 
		{
			continue;
			ros::spinOnce();
		    rate.sleep();
		} 

        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && (input_type == false))
    {
        std::printf("Current GPS position: [%f, %f, %.3f]\n", 
                     global_position.latitude, 
                     global_position.longitude, 
                     global_position.altitude);
        std::printf("Next GPS position: [%f, %f, %.3f]\n", 
                            goal_pos[i+1][0], 
                            goal_pos[i+1][1],
                            goal_pos[i+1][2]);
        distance = measureGPS(global_position.latitude, 
                              global_position.longitude, 
                              global_position.altitude, 
                              goal_pos[i][0], goal_pos[i][1], goal_pos[i][2]);
        std::printf("Distance to next goal: %.2f m \n", distance);
        
        if (i < (goal_num - 1))
        {
            final_check = false;
            enu_goal = WGS84ToENU(goal_pos[i][0], goal_pos[i][1], goal_pos[i][2],
                        refpoint.latitude, refpoint.longitude, refpoint.altitude);
            target_pose.pose.position.x = enu_goal.x;
            target_pose.pose.position.y = enu_goal.y;
            target_pose.pose.position.z = enu_goal.z;
            roll  = radian(0);
            pitch = radian(0);
            yaw   = radian(0);
            q.setRPY(roll, pitch, yaw);
	        tf::quaternionTFToMsg(q, target_pose.pose.orientation);
            target_pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(target_pose);
            
            // distance = measureGPS(global_position.latitude, 
            //                       global_position.longitude, 
            //                       global_position.altitude, 
            //                       goal_pos[i][0], goal_pos[i][1], goal_pos[i][2]);
            // std::printf("Distance to next goal: %.2f m \n", distance);

            ros::spinOnce();
            rate.sleep();
        }
        else
        {
            final_check = true;
            // distance = measureGPS(global_position.latitude, 
            //                       global_position.longitude, 
            //                       global_position.altitude, 
            //                       goal_pos[goal_num - 1][0], 
            //                       goal_pos[goal_num - 1][1], 
            //                       goal_pos[goal_num - 1][2]);
            // std::printf("Distance to next goal: %.2f m \n", distance);

            enu_goal = WGS84ToENU(goal_pos[goal_num-1][0], 
                                  goal_pos[goal_num-1][1], 
                                  goal_pos[goal_num-1][2],
                refpoint.latitude, refpoint.longitude, refpoint.altitude);
            target_pose.pose.position.x = enu_goal.x;
            target_pose.pose.position.y = enu_goal.y;
            target_pose.pose.position.z = enu_goal.z;
            roll  = radian(0);
            pitch = radian(0);
            yaw   = radian(0);
            q.setRPY(roll, pitch, yaw);
	        tf::quaternionTFToMsg(q, target_pose.pose.orientation);
            target_pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(target_pose);
            
            ros::spinOnce();
            rate.sleep();
        }
        enu_curr = WGS84ToENU(global_position.latitude,
                              global_position.longitude,
                              global_position.altitude,
                              refpoint.latitude, 
                              refpoint.longitude, 
                              refpoint.altitude);
        bool check = check_position(enu_goal.x, enu_goal.y, enu_goal.z,
                                    enu_curr.x, enu_curr.y, enu_curr.z);
        std::cout << check << std::endl;
        if (check && !final_check)
        {
            std::printf("[ INFO] Reached position: [%f, %f, %.3f]\n", 
                            global_position.latitude, 
                            global_position.longitude, 
                            global_position.altitude);
            std::printf("Next GPS position: [%f, %f, %.3f]\n", 
                            goal_pos[i+1][0], 
                            goal_pos[i+1][1],
                            goal_pos[i+1][2]);
            files(enu_curr.x, enu_curr.y, enu_curr.z, 
                  global_position.latitude,
                  global_position.longitude,
                  global_position.altitude);
            files(current_pose.pose.position.x, 
                  current_pose.pose.position.y, 
                  current_pose.pose.position.z, 
                    0,
                    0,
                    0);
            ros::Duration(5).sleep();
			i = i + 1;
	    	ros::spinOnce();
    		rate.sleep();
        }
        else if (check && final_check)
        {
            std::printf("[ INFO] Reached FINAL position: [%f, %f, %.3f]\n", 
                            global_position.latitude, 
                            global_position.longitude, 
                            global_position.altitude);
            std::printf("[ INFO] Ready to LANDING \n");
            files(enu_curr.x, enu_curr.y, enu_curr.z, 
                  global_position.latitude,
                  global_position.longitude,
                  global_position.altitude);
            files(current_pose.pose.position.x, 
                  current_pose.pose.position.y, 
                  current_pose.pose.position.z, 
                    0,
                    0,
                    0);
            ros::Duration(5).sleep();

			set_mode.request.custom_mode = "AUTO.LAND";
    	    if( set_mode_client.call(set_mode) && set_mode.response.mode_sent)
            {
    		    std::cout << "[ INFO] AUTO.LAND enabled \n";
                break;
            }
        }
        else 
		{
			continue;
			ros::spinOnce();
		    rate.sleep();
		} 

        ros::spinOnce();
        rate.sleep();
    }

    // ros::spinOnce();
    // rate.sleep();

    return 0;
}