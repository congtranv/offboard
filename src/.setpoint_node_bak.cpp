// if (input_type == true) // local setpoint
    // {
    //     int i = 0;
    //     while (ros::ok())
    //     {
            // std::printf("Current local position: [%.3f, %.3f, %.3f]\n", 
            //          current_pose.pose.position.x, 
            //          current_pose.pose.position.y, 
            //          current_pose.pose.position.z);
    		
            // batt_percent = current_batt.percentage * 100;
            // std::printf("Current Battery capacity: %.1f \n", batt_percent);

            // if (i < (target_num -1))
            // {
            //     final_check = false;
            //     target_pose.pose.position.x = target_pos[i][0];
            //     target_pose.pose.position.y = target_pos[i][1];
            //     target_pose.pose.position.z = target_pos[i][2];
            //     roll  = radian(target_pos[i][3]);
            //     pitch = radian(target_pos[i][4]);
            //     yaw   = radian(target_pos[i][5]);
            //     q.setRPY(roll, pitch, yaw);
	        //     tf::quaternionTFToMsg(q, target_pose.pose.orientation);

            //     target_pose.header.stamp = ros::Time::now();
            //     local_pos_pub.publish(target_pose);
    		// 	ros::spinOnce();
            // 	rate.sleep();
            // }
            // else
            // {
            //     final_check = true;
            //     target_pose.pose.position.x = target_pos[target_num - 1][0];
            //     target_pose.pose.position.y = target_pos[target_num - 1][1];
            //     target_pose.pose.position.z = target_pos[target_num - 1][2];
            //     roll  = radian(target_pos[target_num - 1][3]);
            //     pitch = radian(target_pos[target_num - 1][4]);
            //     yaw   = radian(target_pos[target_num - 1][5]);
            //     q.setRPY(roll, pitch, yaw);
	        //     tf::quaternionTFToMsg(q, target_pose.pose.orientation);

            //     target_pose.header.stamp = ros::Time::now();
            //     local_pos_pub.publish(target_pose);
    		// 	ros::spinOnce();
            // 	rate.sleep();
            // }

    //         bool check = check_position(target_pose.pose.position.x,
    //                                     target_pose.pose.position.y,
    //                                     target_pose.pose.position.z,
    //                                     current_pose.pose.position.x,
    //                                     current_pose.pose.position.y,
    //                                     current_pose.pose.position.z);
    // 		std::cout << check << std::endl; 
    //         if (check)
    //         {
    //             if (!final_check)
    //             {
    //                 std::printf("Reached position: [%.3f, %.3f, %.3f]\n", 
    //                         current_pose.pose.position.x, 
    //                         current_pose.pose.position.y, 
    //                         current_pose.pose.position.z);
    		        
    //                 std::printf("Next position: [%.3f, %.3f, %.3f]\n", 
    //                                 target_pos[i+1][0], 
    //                                 target_pos[i+1][1],
    //                                 target_pos[i+1][2]);
                    
    //                 ros::Duration(5).sleep();
	// 		        i = i + 1;
    //                 ros::spinOnce();
    // 		        rate.sleep();
    //             }
    //             else
    //             {
    //                 std::printf("Reached final position: [%.3f, %.3f, %.3f]\n", 
    //                         current_pose.pose.position.x, 
    //                         current_pose.pose.position.y, 
    //                         current_pose.pose.position.z);
    		        
    //                 std::printf("Ready to landing \n");
    //                 ros::Duration(5).sleep();

	// 		        set_mode.request.custom_mode = "AUTO.LAND";
    //                 while(ros::ok()){
    // 	                if( set_mode_client.call(set_mode) && set_mode.response.mode_sent){
    // 		                std::cout << "[ INFO] AUTO.LAND enabled \n";
    //                         break;
    // 	                }
    // 	                break;
    //                 }
	// 		        // break;
    //                 ros::spinOnce();
    // 		        rate.sleep();
    //             }
    //             ros::spinOnce();
    // 		    rate.sleep();
                
    //         }
    //         else
    //         {
    //             continue;
    //             ros::spinOnce();
    //             rate.sleep();
    //         }

    //         ros::spinOnce();
    //         rate.sleep();
    //     }
    //     ros::spinOnce();
    // 	rate.sleep();
    // }
    // else // glocal setpoint
    // {
    //     int i = 0;
    //     while (ros::ok())
    //     {
    //         std::printf("Current GPS position: [%f, %f, %.3f]\n", 
    //                  global_position.latitude, 
    //                  global_position.longitude, 
    //                  global_position.altitude);
    //         batt_percent = current_batt.percentage * 100;
    //         std::printf("Current Battery capacity: %.1f \n", batt_percent);

    //         if (i < (goal_num - 1))
    //         {
    //             final_check = false;
    //             enu_goal = WGS84ToENU(goal_pos[i][0], goal_pos[i][1], goal_pos[i][2],
    //                         refpoint.latitude, refpoint.longitude, refpoint.altitude);
    //             target_pose.pose.position.x = enu_goal.x;
    //             target_pose.pose.position.y = enu_goal.y;
    //             target_pose.pose.position.z = enu_goal.z;
    //             roll  = radian(0);
    //             pitch = radian(0);
    //             yaw   = radian(0);
    //             q.setRPY(roll, pitch, yaw);
	//             tf::quaternionTFToMsg(q, target_pose.pose.orientation);
    //             target_pose.header.stamp = ros::Time::now();
    //             local_pos_pub.publish(target_pose);
            
    //             distance = measureGPS(global_position.latitude, 
    //                                 global_position.longitude, 
    //                                 global_position.altitude, 
    //                                 goal_pos[i][0], goal_pos[i][1], goal_pos[i][2]);
    //             std::printf("Distance to next goal: %.2f m \n", distance);

    //             ros::spinOnce();
    //             rate.sleep();
    //         }
    //         else
    //         {
    //             final_check = true;
    //             distance = measureGPS(global_position.latitude, 
    //                                   global_position.longitude, 
    //                                   global_position.altitude, 
    //                                   goal_pos[goal_num - 1][0], 
    //                                   goal_pos[goal_num - 1][1], 
    //                                   goal_pos[goal_num - 1][2]);
    //             std::printf("Distance to next goal: %.2f m \n", distance);

    //             enu_goal = WGS84ToENU(goal_pos[goal_num-1][0], 
    //                                   goal_pos[goal_num-1][1], 
    //                                   goal_pos[goal_num-1][2],
    //                 refpoint.latitude, refpoint.longitude, refpoint.altitude);
    //             target_pose.pose.position.x = enu_goal.x;
    //             target_pose.pose.position.y = enu_goal.y;
    //             target_pose.pose.position.z = enu_goal.z;
    //             roll  = radian(0);
    //             pitch = radian(0);
    //             yaw   = radian(0);
    //             q.setRPY(roll, pitch, yaw);
	//             tf::quaternionTFToMsg(q, target_pose.pose.orientation);
    //             target_pose.header.stamp = ros::Time::now();
    //             local_pos_pub.publish(target_pose);
            
    //             ros::spinOnce();
    //             rate.sleep();
    //         }

    //         enu_curr = WGS84ToENU(global_position.latitude,
    //                               global_position.longitude,
    //                               global_position.altitude,
    //                               refpoint.latitude, 
    //                               refpoint.longitude, 
    //                               refpoint.altitude);
    //         bool check = check_position(enu_goal.x, enu_goal.y, enu_goal.z,
    //                                     enu_curr.x, enu_curr.y, enu_curr.z);
    //         std::cout << check << std::endl;
    //         if (check)
    //         {
    //             if (!final_check)
    //             {
    //                 std::printf("Reached position: [%f, %f, %.3f]\n", 
    //                         global_position.latitude, 
    //                         global_position.longitude, 
    //                         global_position.altitude);
    //                 // files("setpoint", enu_curr.x, enu_curr.y, enu_curr.z,
    //                 //             global_position.latitude, 
    //                 //             global_position.longitude,
    //                 //             global_position.altitude);
    //                 // files("setpoint", 0, 0, 0, 0, 0, 0);
    //                 std::printf("Next position: [%f, %f, %.3f]\n", 
    //                             goal_pos[i+1][0], 
    //                             goal_pos[i+1][1],
    //                             goal_pos[i+1][2]);

    //                 ros::Duration(5).sleep();
	// 		        i = i + 1;
	//     		    ros::spinOnce();
    // 		        rate.sleep();
    //             }
    //             else
    //             {
    //                 std::printf("Reached final position: [%f, %f, %.3f]\n", 
    //                         global_position.latitude, 
    //                         global_position.longitude, 
    //                         global_position.altitude);
    //                 // files("setpoint", enu_curr.x, enu_curr.y, enu_curr.z,
    //                 //             global_position.latitude, 
    //                 //             global_position.longitude,
    //                 //             global_position.altitude);
    //                 // files("setpoint", 0, 0, 0, 0, 0, 0);
    //                 std::printf("Ready to landing \n");
    //                 ros::Duration(5).sleep();

	// 		        set_mode.request.custom_mode = "AUTO.LAND";
    //                 while(ros::ok()){
    // 	                if( set_mode_client.call(set_mode) && set_mode.response.mode_sent){
    // 		                std::cout << "[ INFO] AUTO.LAND enabled \n";
    //                         break;
    // 	                }
    // 	                break;
    //                 }
	//     		    break;
    //                 ros::spinOnce();
    // 		        rate.sleep();
    //             }
    //             ros::spinOnce();
    // 		    rate.sleep();
    //         }
    //         else
    //         {
    //             continue;
    //             ros::spinOnce();
    //             rate.sleep();
    //         }

    //         ros::spinOnce();
    //         rate.sleep();
    //     }
    //     ros::spinOnce();
    // 	rate.sleep();
    // }
