#include "offboard/offboard.h"

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "local");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, localPose_cb);
    ros::Subscriber rel_alt_sub = nh.subscribe<std_msgs::Float64>
            ("mavros/global_position/rel_alt", 10, relativeAlt_cb);
    ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data_raw", 10, imuData_cb);
    ros::Subscriber mag_data_sub = nh.subscribe<sensor_msgs::MagneticField>
            ("mavros/imu/mag", 10, magData_cb);
    ros::Subscriber static_press_sub = nh.subscribe<sensor_msgs::FluidPressure>
            ("mavros/imu/static_pressure", 10, staticPress_cb);
    ros::Subscriber diff_press_sub = nh.subscribe<sensor_msgs::FluidPressure>
            ("mavros/imu/diff_pressure", 10, diffPress_cb);

    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix> 
            ("mavros/global_position/global", 10, globalPosition_cb);
    ros::Subscriber gps_pos_sub = nh.subscribe<mavros_msgs::GPSRAW> 
            ("mavros/gpsstatus/gps1/raw", 10, gpsPosition_cb);

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
    std::cout << "[ INFO] Checking status...\n";
    ros::Duration(1).sleep();

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
        
        std::cout << "Current relative altitude: " << rel_alt.data << " m \n";

        batt_percent = current_batt.percentage * 100;
        std::printf("Current Battery capacity: %.1f \n", batt_percent);
        std::cout << "\n";
		ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Check status done \n";

    creates();
    creates_sensor();

    gps_lat = double(gps_position.lat)/10000000;
    gps_lon = double(gps_position.lon)/10000000;
    gps_alt = double(gps_position.alt)/1000;
    updates("initial",  current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z,
                        global_position.latitude,
                        global_position.longitude,
                        global_position.altitude,
                        gps_lat, gps_lon, gps_alt, 
                        rel_alt.data);
    updates_sensor("initial", imu_data.angular_velocity.x, 
                              imu_data.angular_velocity.y, 
                              imu_data.angular_velocity.z,
                              imu_data.linear_acceleration.x, 
                              imu_data.linear_acceleration.y, 
                              imu_data.linear_acceleration.z,
                              mag_data.magnetic_field.x, 
                              mag_data.magnetic_field.y, 
                              mag_data.magnetic_field.z,
                              static_press.fluid_pressure, 
                              diff_press.fluid_pressure);

    input_local_target();
    target_pose.pose.position.x = target_pos[0][0];
    target_pose.pose.position.y = target_pos[0][1];
    target_pose.pose.position.z = target_pos[0][2];

    std::cout << "[ INFO] Setting OFFBOARD stream...\n";
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
    ros::Duration(1).sleep();

    gps_lat = double(gps_position.lat)/10000000;
    gps_lon = double(gps_position.lon)/10000000;
    gps_alt = double(gps_position.alt)/1000;

    updates("pre-flight", current_pose.pose.position.x,
                          current_pose.pose.position.y,
                          current_pose.pose.position.z,
                          global_position.latitude,
                          global_position.longitude,
                          global_position.altitude,
                          gps_lat, gps_lon, gps_alt, 
                          rel_alt.data);
    updates_sensor("pre-flight", imu_data.angular_velocity.x, 
                                 imu_data.angular_velocity.y,
                                 imu_data.angular_velocity.z,
                                 imu_data.linear_acceleration.x, 
                                 imu_data.linear_acceleration.y, 
                                 imu_data.linear_acceleration.z,
                                 mag_data.magnetic_field.x, 
                                 mag_data.magnetic_field.y, 
                                 mag_data.magnetic_field.z,
                                 static_press.fluid_pressure, 
                                 diff_press.fluid_pressure);
    int i = 0;
    while (ros::ok())
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
            
            gps_lat = double(gps_position.lat)/10000000;
            gps_lon = double(gps_position.lon)/10000000;
            gps_alt = double(gps_position.alt)/1000;
            updates_check(i+1, current_pose.pose.position.x,
                               current_pose.pose.position.y,
                               current_pose.pose.position.z,
                               global_position.latitude,
                               global_position.longitude,
                               global_position.altitude,
                               gps_lat, gps_lon, gps_alt, rel_alt.data);
            updates_check_ss(i+1, imu_data.angular_velocity.x, 
                                  imu_data.angular_velocity.y,
                                  imu_data.angular_velocity.z,
                                  imu_data.linear_acceleration.x, 
                                  imu_data.linear_acceleration.y, 
                                  imu_data.linear_acceleration.z,
                                  mag_data.magnetic_field.x, 
                                  mag_data.magnetic_field.y, 
                                  mag_data.magnetic_field.z,
                                  static_press.fluid_pressure, 
                                  diff_press.fluid_pressure);
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
           
            gps_lat = double(gps_position.lat)/10000000;
            gps_lon = double(gps_position.lon)/10000000;
            gps_alt = double(gps_position.alt)/1000;
            updates_check(i+1, current_pose.pose.position.x,
                               current_pose.pose.position.y,
                               current_pose.pose.position.z,
                               global_position.latitude,
                               global_position.longitude,
                               global_position.altitude,
                               gps_lat, gps_lon, gps_alt, rel_alt.data);
            updates_check_ss(i+1, imu_data.angular_velocity.x, 
                                  imu_data.angular_velocity.y,
                                  imu_data.angular_velocity.z,
                                  imu_data.linear_acceleration.x, 
                                  imu_data.linear_acceleration.y, 
                                  imu_data.linear_acceleration.z,
                                  mag_data.magnetic_field.x, 
                                  mag_data.magnetic_field.y, 
                                  mag_data.magnetic_field.z,
                                  static_press.fluid_pressure, 
                                  diff_press.fluid_pressure);
            ros::Duration(5).sleep();
            
			// set_mode.request.custom_mode = "AUTO.LAND";
    	    // if( set_mode_client.call(set_mode) && set_mode.response.mode_sent)
            // {
    		//     std::cout << "[ INFO] AUTO.LAND enabled \n";
            //     break;
            // }
            i = target_num -1;
            ros::spinOnce();
		    rate.sleep();
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

    return 0;
}