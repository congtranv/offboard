#include <offboard/offboard.h>
#include <offboard/logging.h>

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber batt_sub = nh.subscribe<sensor_msgs::BatteryState> 
            ("mavros/battery", 10, battery_cb);

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, localPose_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix> 
            ("mavros/global_position/global", 10, globalPosition_cb);
    ros::Subscriber gps_pos_sub = nh.subscribe<mavros_msgs::GPSRAW> 
            ("mavros/gpsstatus/gps1/raw", 10, gpsPosition_cb);
    ros::Subscriber rel_alt_sub = nh.subscribe<std_msgs::Float64>
            ("mavros/global_position/rel_alt", 10, relativeAlt_cb);
    
    // service
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

    // publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    std::cout << "[ INFO] Waiting for FCU connection...\n";
    while(ros::ok() && !current_state.connected)
	{
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] FCU connected \n";

    // wait for GPS information
    std::cout << "[ INFO] Waiting for GPS signal...\n";
    while (ros::ok() && !global_position_received && !gps_position_received) 
    {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] GPS position received \n";

    creates(); // open position log (.csv) file
    creates_sensor(); // open sensors data (.csv) file
    zero_pose.pose.position.x = 0;
    zero_pose.pose.position.y = 0;
    zero_pose.pose.position.z = 0;
    zero_point.x = 0;
    zero_point.y = 0;
    zero_point.z = 0;

    std::cout << "[ INFO] Waiting for stable initial... \n";
    t_check = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - t_check) < ros::Duration(20))
    {
        updates("stabilizing", current_pose, zero_point, global_position,
                               gps_position, rel_alt.data);
        updates_sensor("stabilizing", imu_data, mag_data, static_press, diff_press);
        ros::Duration(0.1).sleep();
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Stable initial done \n";
    
    // init reference point
    refpoint.latitude = global_position.latitude;
    refpoint.longitude = global_position.longitude;
    refpoint.altitude = global_position.altitude;

    std::printf("\nCurrent global position: [%f, %f, %.3f]\n", 
                global_position.latitude, 
                global_position.longitude, 
                global_position.altitude);
    std::cout << "Current relative altitude: " << rel_alt.data << " m \n";
        
    std::printf("Reference GPS position: [%f, %f, %.3f]\n", 
                    refpoint.latitude, 
                    refpoint.longitude, 
                    refpoint.altitude);
    enu_curr = WGS84ToENU(global_position.latitude,
                        global_position.longitude,
                        global_position.altitude,
                        refpoint.latitude, 
                        refpoint.longitude, 
                        refpoint.altitude);
    enu_ref = enu_curr;
    std::printf("Current local position: [%.3f, %.3f, %.3f]\n", 
                current_pose.pose.position.x, 
                current_pose.pose.position.y, 
                current_pose.pose.position.z);
    std::printf("Current converted position: [%.3f, %.3f, %.3f]\n", 
                enu_curr.x, 
                enu_curr.y, 
                enu_curr.z);

    updates("initial",  current_pose, enu_curr, global_position, 
                        gps_position, rel_alt.data);
    updates("reference", current_pose, enu_ref, refpoint,
                         gps_position, rel_alt.data);
    updates_sensor("initial", imu_data, mag_data, static_press, diff_press);
    
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        enu_curr = WGS84ToENU(global_position.latitude,
                            global_position.longitude,
                            global_position.altitude,
                            refpoint.latitude, 
                            refpoint.longitude, 
                            refpoint.altitude);
        x_off[i] = current_pose.pose.position.x - enu_curr.x;
        y_off[i] = current_pose.pose.position.y - enu_curr.y;
        z_off[i] = current_pose.pose.position.z - enu_curr.z;

        ros::spinOnce();
        rate.sleep();
    }
    for(int i = 100; i > 0; --i)
    {
        offset.x = offset.x + x_off[i]/100;
        offset.y = offset.y + y_off[i]/100;
        offset.z = offset.z + z_off[i]/100;
    }
    std::printf("\nOffset: [%f, %f, %f]\n", offset.x, offset.y, offset.z);
    updates("Offset", current_pose, offset, global_position, gps_position, rel_alt.data);
     
    // set target pose
    input_target();
    if (local_input == true) // local setpoint
    {
        target_pose.pose.position.x = in_x_pos[0];
        target_pose.pose.position.y = in_y_pos[0];
        target_pose.pose.position.z = in_z_pos[0];
    }
    else // global setpoint
    {
        enu_goal = WGS84ToENU(in_latitude[0], 
                              in_longitude[0], 
                              in_altitude[0],
                              refpoint.latitude, 
                              refpoint.longitude, 
                              refpoint.altitude);
        target_pose.pose.position.x = enu_goal.x + offset.x;
        target_pose.pose.position.y = enu_goal.y + offset.y;
        target_pose.pose.position.z = enu_goal.z + offset.z;
    }

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

    enu_curr = WGS84ToENU(global_position.latitude,
                          global_position.longitude,
                          global_position.altitude,
                          refpoint.latitude, 
                          refpoint.longitude, 
                          refpoint.altitude);
    updates("pre-flight", current_pose, enu_curr, global_position, gps_position, rel_alt.data);
    updates_sensor("pre-flight", imu_data, mag_data, static_press, diff_press);
    
    std::cout << "[ INFO] Waiting arm and takeoff... \n";
    while (ros::ok() && !current_state.armed)
    {
        ros::spinOnce();
        rate.sleep();
    }

    system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
	ros::param::get("xt", takeoff_pose.pose.position.x);
	ros::param::get("yt", takeoff_pose.pose.position.y);
	ros::param::get("zt", takeoff_pose.pose.position.z);
    ros::param::get("hover_time", t_hover);
    printf("Takeoff position: %.3f %.3f %.3f, Hover time %.3f \n", takeoff_pose.pose.position.x,
            takeoff_pose.pose.position.y, takeoff_pose.pose.position.z, t_hover);
    
    std::cout << "[ INFO] Takeoff... \n";
    bool takeoff_check = false;
    while (ros::ok() && !takeoff_check)
    {
        takeoff_pose.header.stamp = ros::Time::now();   
        local_pos_pub.publish(takeoff_pose);
                    
        takeoff_check = check_position(check_error, current_pose, takeoff_pose);
        // printf("takeoff check: %d\n",takeoff_check);
        t_check = ros::Time::now();
        if (takeoff_check)
        {
            while ((ros::Time::now() - t_check) < ros::Duration(t_hover))
            {
                local_pos_pub.publish(takeoff_pose);

                ros::spinOnce();
    		    rate.sleep();
            }
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
        
    }
    // printf("end takeoff\n");
    int i = 0; 
    while (ros::ok() && local_input)
    { 
        if (i < (target_num -1))
        {
            final_check = false;
            target_pose.pose.position.x = in_x_pos[i];
            target_pose.pose.position.y = in_y_pos[i];
            target_pose.pose.position.z = in_z_pos[i];
            printf("target %d: %.3f %.3f %.3f", i, in_x_pos[i], in_y_pos[i], in_z_pos[i]);

            bool traj_check = false;
            traj_pose = current_pose;
            while (!traj_check)
            {
                velociyGenerate(traj_pose, target_pose, &v_x, &v_y, &v_z);
                traj_pose.pose.position.x = traj_pose.pose.position.x + v_x;
                traj_pose.pose.position.y = traj_pose.pose.position.y + v_y;
                traj_pose.pose.position.z = traj_pose.pose.position.z + v_z;
                traj_pose.header.stamp = ros::Time::now();

                local_pos_pub.publish(traj_pose);
                traj_check = check_position(check_error, traj_pose, target_pose);

                ros::spinOnce();
                rate.sleep();
            }
            local_pos_pub.publish(target_pose);
            ros::spinOnce();
            rate.sleep();
        }
        else
        {
            final_check = true;
            target_pose.pose.position.x = in_x_pos[target_num - 1];
            target_pose.pose.position.y = in_y_pos[target_num - 1];
            target_pose.pose.position.z = in_z_pos[target_num - 1];

            printf("target %d: %f %f %f \n", i, in_x_pos[target_num - 1], in_y_pos[target_num - 1], in_z_pos[target_num - 1]);
            velociyGenerate(current_pose, target_pose, &v_x, &v_y, &v_z);
            printf("vx = %f, vy = %f, vz = %f \n", v_x, v_y, v_z);
            
            traj_pose = current_pose;
            bool traj_check = false;
            while (!traj_check)
            {
                traj_pose.pose.position.x = traj_pose.pose.position.x + v_x;
                traj_pose.pose.position.y = traj_pose.pose.position.y + v_y;
                traj_pose.pose.position.z = traj_pose.pose.position.z + v_z;
                local_pos_pub.publish(traj_pose);
                
                traj_check = check_position(check_error, traj_pose, target_pose);
                printf("traj check %d\n", traj_check);
                    
                t_check = ros::Time::now();
                if (traj_check)
                {
                    while ((ros::Time::now() - t_check) < ros::Duration(3))
                    {
                        local_pos_pub.publish(target_pose);
                        ros::spinOnce();
                        rate.sleep();
                    }  
                }
                  
                enu_curr = WGS84ToENU(global_position.latitude,
                                    global_position.longitude,
                                    global_position.altitude,
                                    refpoint.latitude, 
                                    refpoint.longitude, 
                                    refpoint.altitude);
                updates("flight", current_pose, enu_curr, global_position, gps_position, rel_alt.data);
                updates_sensor("flight", imu_data, mag_data, static_press, diff_press);
                    
                ros::spinOnce();
                rate.sleep();
            }
                
            // ros::spinOnce();
            // rate.sleep();
        }

        std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", 
                        current_pose.pose.position.x, 
                        current_pose.pose.position.y, 
                        current_pose.pose.position.z);
        std::printf("Target local position: [%.3f, %.3f, %.3f]\n", 
                            in_x_pos[i], in_y_pos[i], in_z_pos[i]);
        distance = distanceLocal(current_pose, target_pose);
        std::printf("Distance to target: %.3f m \n", distance);

        bool check = check_position(check_error, current_pose, target_pose);
        std::cout << "\n" << check << std::endl;
        if(check && !final_check)
        {
            t_check = ros::Time::now();
            std::printf("[ INFO] Reached position: [%.3f, %.3f, %.3f]\n", 
                            current_pose.pose.position.x, 
                            current_pose.pose.position.y, 
                            current_pose.pose.position.z);   
            std::printf("[ INFO] Next local position: [%.3f, %.3f, %.3f]\n", 
                            in_x_pos[i+1], in_y_pos[i+1], in_z_pos[i+1]);
            std::cout << "[ INFO] Hover at checkpoint \n";

            while ((ros::Time::now() - t_check) < ros::Duration(t_hover))
            {
                local_pos_pub.publish(target_pose);

                enu_curr = WGS84ToENU(global_position.latitude,
                        global_position.longitude,
                        global_position.altitude,
                        refpoint.latitude, 
                        refpoint.longitude, 
                        refpoint.altitude);
                updates_check(i+1, current_pose, enu_curr, global_position, gps_position, rel_alt.data);
                updates_check_ss(i+1, imu_data, mag_data, static_press, diff_press);

                ros::spinOnce();
    		    rate.sleep();
            }

            i = i + 1;
            ros::spinOnce();
    		rate.sleep();
    	}
        else if (check && final_check)
        {
            t_check = ros::Time::now();
            std::printf("[ INFO] Reached FINAL position: [%.3f, %.3f, %.3f]\n", 
                            current_pose.pose.position.x, 
                            current_pose.pose.position.y, 
                            current_pose.pose.position.z);
            std::printf("[ INFO] Ready to LANDING \n");
            
            while ((ros::Time::now() - t_check) < ros::Duration(t_hover))
            {
                local_pos_pub.publish(target_pose);

                enu_curr = WGS84ToENU(global_position.latitude,
                        global_position.longitude,
                        global_position.altitude,
                        refpoint.latitude, 
                        refpoint.longitude, 
                        refpoint.altitude);
                updates_check(i+1, current_pose, enu_curr, global_position, gps_position, rel_alt.data);
                updates_check_ss(i+1, imu_data, mag_data, static_press, diff_press);

                ros::spinOnce();
    		    rate.sleep();
            }

    		set_mode.request.custom_mode = "AUTO.LAND";
        	if( set_mode_client.call(set_mode) && set_mode.response.mode_sent)
            {
        		std::cout << "[ INFO] AUTO.LAND enabled \n";
                break;
            }

            ros::spinOnce();
    		rate.sleep();
        }
    	else 
    	{
    		ros::spinOnce();
    		rate.sleep();
    	} 

        // ros::spinOnce();
        // rate.sleep();
    }
    
    while (ros::ok() && !local_input)
    {
        if (i < (goal_num - 1))
        {
            final_check = false;
            enu_goal = WGS84ToENU(in_latitude[i], 
                                    in_longitude[i], 
                                    in_altitude[i],
                                    refpoint.latitude, 
                                    refpoint.longitude, 
                                    refpoint.altitude);
            target_pose.pose.position.x = enu_goal.x + offset.x;
            target_pose.pose.position.y = enu_goal.y + offset.y;
            target_pose.pose.position.z = enu_goal.z + offset.z;
                
            target_pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(target_pose);

            enu_curr = WGS84ToENU(global_position.latitude,
                                global_position.longitude,
                                global_position.altitude,
                                refpoint.latitude, 
                                refpoint.longitude, 
                                refpoint.altitude);
            updates("flight", current_pose, enu_curr, global_position, gps_position, rel_alt.data);
            updates_sensor("flight", imu_data, mag_data, static_press, diff_press);

            // ros::spinOnce();
            // rate.sleep();
        }
        else
        {
            final_check = true;
            enu_goal = WGS84ToENU(in_latitude[goal_num-1], 
                                    in_longitude[goal_num-1], 
                                    in_altitude[goal_num-1],
                                    refpoint.latitude, 
                                    refpoint.longitude, 
                                    refpoint.altitude);
            target_pose.pose.position.x = enu_goal.x + offset.x;
            target_pose.pose.position.y = enu_goal.y + offset.y;
            target_pose.pose.position.z = enu_goal.z + offset.z;
                
            target_pose.header.stamp = ros::Time::now();
            local_pos_pub.publish(target_pose);

            enu_curr = WGS84ToENU(global_position.latitude,
                                global_position.longitude,
                                global_position.altitude,
                                refpoint.latitude, 
                                refpoint.longitude, 
                                refpoint.altitude);
            updates("flight", current_pose, enu_curr, global_position, gps_position, rel_alt.data);
            updates_sensor("flight", imu_data, mag_data, static_press, diff_press);
                
            // ros::spinOnce();
            // rate.sleep();
        }

        std::printf("\nCurrent GPS position: [%.8f, %.8f, %.3f]\n", 
                    global_position.latitude, 
                    global_position.longitude, 
                    global_position.altitude);
        std::printf("Goal GPS position: [%.8f, %.8f, %.3f]\n", 
                            in_latitude[i], 
                            in_longitude[i],
                            in_altitude[i]);
           
        std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", 
                        current_pose.pose.position.x, 
                        current_pose.pose.position.y, 
                        current_pose.pose.position.z);
        std::printf("Target local position: [%.3f, %.3f, %.3f]\n", 
                            target_pose.pose.position.x, 
                            target_pose.pose.position.y,
                            target_pose.pose.position.z);
        distance = distanceLocal(current_pose, target_pose);
        std::printf("Distance to target: %.3f m \n", distance);

        bool check = check_position(check_error,current_pose, target_pose);

        std::cout << "\n" << check << std::endl;
        if (check && !final_check)
        {
            t_check = ros::Time::now();
            std::printf("[ INFO] Reached position: [%.8f, %.8f, %.3f]\n", 
                            global_position.latitude, 
                            global_position.longitude, 
                            global_position.altitude);
            std::printf("[ INFO] Next GPS position: [%.8f, %.8f, %.3f]\n", 
                            in_latitude[i+1], 
                            in_longitude[i+1],
                            in_altitude[i+1]);
            std::printf("[ INFO] Reached position: [%.3f, %.3f, %.3f]\n", 
                            current_pose.pose.position.x, 
                            current_pose.pose.position.y, 
                            current_pose.pose.position.z);   
            std::printf("[ INFO] Target local position: [%.3f, %.3f, %.3f]\n", 
                            target_pose.pose.position.x, 
                            target_pose.pose.position.y,
                            target_pose.pose.position.z);
            std::cout << "[ INFO] Hover at checkpoint \n";

            while ((ros::Time::now() - t_check) < ros::Duration(t_hover))
            {
                local_pos_pub.publish(target_pose);

                enu_curr = WGS84ToENU(global_position.latitude,
                        global_position.longitude,
                        global_position.altitude,
                        refpoint.latitude, 
                        refpoint.longitude, 
                        refpoint.altitude);
                updates_check(i+1, current_pose, enu_curr, global_position, gps_position, rel_alt.data);
                updates_check_ss(i+1, imu_data, mag_data, static_press, diff_press);

                ros::spinOnce();
    		    rate.sleep();
            }

            i = i + 1;
            ros::spinOnce();
            rate.sleep();
        }
        else if (check && final_check)
        {
            t_check = ros::Time::now();
            std::printf("[ INFO] Reached FINAL position: [%.8f, %.8f, %.3f]\n", 
                            global_position.latitude, 
                            global_position.longitude, 
                            global_position.altitude);
            std::printf("[ INFO] Reached local position: [%.3f, %.3f, %.3f]\n", 
                            current_pose.pose.position.x, 
                            current_pose.pose.position.y, 
                            current_pose.pose.position.z);
            std::printf("[ INFO] Target local position: [%.3f, %.3f, %.3f]\n", 
                            target_pose.pose.position.x, 
                            target_pose.pose.position.y,
                            target_pose.pose.position.z);
            std::printf("[ INFO] Ready to LANDING \n");

            while ((ros::Time::now() - t_check) < ros::Duration(t_hover))
            {
                local_pos_pub.publish(target_pose);

                enu_curr = WGS84ToENU(global_position.latitude,
                        global_position.longitude,
                        global_position.altitude,
                        refpoint.latitude, 
                        refpoint.longitude, 
                        refpoint.altitude);
                updates_check(i+1, current_pose, enu_curr, global_position, gps_position, rel_alt.data);
                updates_check_ss(i+1, imu_data, mag_data, static_press, diff_press);

                ros::spinOnce();
    		    rate.sleep();
            }

            set_mode.request.custom_mode = "AUTO.LAND";
            if( set_mode_client.call(set_mode) && set_mode.response.mode_sent)
            {
                std::cout << "[ INFO] AUTO.LAND enabled \n";
                break;
            }

            ros::spinOnce();
            rate.sleep();
        }
        else 
        {
            ros::spinOnce();
            rate.sleep();
        } 

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
