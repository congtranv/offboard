#include <offboard/offboard.h>

void stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = *msg;
}

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_global_ = *msg;
    global_received_ = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 50, stateCallback);
    local_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 50, localPoseCallback);
    global_pos_sub_ = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 50, globalPositionCallback);

    local_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 100);
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && !current_state_.connected)
	{
        std::printf("\n[ INFO] Waiting for FCU connection \n");
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("\n[ INFO] FCU connected \n");
    // wait for GPS information
    while (ros::ok() && !global_received_) 
    {
        std::printf("[ INFO] Waiting for GPS signal \n");
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("\n[ INFO] GPS position received \n");
	
    std::printf("\n[ INFO] How do you want to fly?\n");
    char c;
    std::printf("- Choose 1 for HOVERING at a Z height\n");
    std::printf("- Choose 2 to fly with SETPOINTs\n");
    std::printf("(1/2): ");
    std::cin >> c;
    if(c == '1')
    {
        std::printf("[ INFO] Hover at a Z height\n");
        std::printf("z = ");
        double x_hover, y_hover, z_hover;
        std::cin >> z_hover;
        x_hover = current_pose_.pose.position.x;
        y_hover = current_pose_.pose.position.y;
        target_pose_ = targetTransfer(x_hover, y_hover, z_hover);
        // send a few setpoints before starting
        std::printf("[ INFO] Setting OFFBOARD stream \n");
        for(int i = 100; ros::ok() && i > 0; --i)
        {
            target_pose_.header.stamp = ros::Time::now();
            local_pose_pub_.publish(target_pose_);
            ros::spinOnce();
            rate.sleep();
        }
        std::printf("\n[ INFO] Set OFFBOARD stream done \n");
        std::printf("[ INFO] Waiting OFFBOARD switching \n");
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD"))
        {
            ros::spinOnce();
            rate.sleep();
        }
        std::printf("\n[ INFO] --------------- READY --------------- \n");
        while(ros::ok())
        {
            vel_ = velLimit(1.0, current_pose_, targetTransfer(x_hover, y_hover, z_hover));

            target_pose_ = targetTransfer(current_pose_.pose.position.x + vel_[0], current_pose_.pose.position.y + vel_[1], current_pose_.pose.position.z + vel_[2]);
            target_pose_.header.stamp = ros::Time::now();
            local_pose_pub_.publish(target_pose_);

            check_error_ = 0.1;
            bool hover_reached = checkPosition(check_error_, current_pose_, targetTransfer(x_hover, y_hover, z_hover));
            if(hover_reached)
            {
                ros::param::get("hover_time", hover_time_);
                std::printf("- Hover at %.3f(m) in %.3f(s)\n", z_hover, hover_time_);
                hoverAt(hover_time_, targetTransfer(x_hover, y_hover, z_hover), rate);
                
                bool landing_reached = false;
                std::printf("[ INFO] Ready to Land\n");
                while(ros::ok() && !landing_reached)
                {
                    ros::param::get("land_velocity", vel_desired_);
                    vel_ = velLimit(vel_desired_, current_pose_, targetTransfer(x_hover, y_hover, 0));

                    target_pose_ = targetTransfer(current_pose_.pose.position.x + vel_[0], current_pose_.pose.position.y + vel_[1], current_pose_.pose.position.z + vel_[2]);

                    target_pose_.header.stamp = ros::Time::now();
                    local_pose_pub_.publish(target_pose_);

                    landing_reached = checkPosition(check_error_, current_pose_, targetTransfer(x_hover, y_hover, 0));
                    if(current_state_.system_status == 3)
                    {
                        std::printf("[ INFO] Landing detected\n");
                        flight_mode_.request.custom_mode = "AUTO.LAND";
                        if(set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent)
                        {
                            break;
                        }
                    }
                    else if(landing_reached)
                    {
                        flight_mode_.request.custom_mode = "AUTO.LAND";
                        if(set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent)
                        {
                            std::printf("[ INFO] LAND\n");
                        }
                    }
                    else
                    {
                        ros::spinOnce();
                        rate.sleep();
                    }
                }
                break;
            }
            
            ros::spinOnce();
            rate.sleep();
        }
    }
    else if(c == '2')
    {
        std::printf("[ INFO] Fly with Setpoints\n");
        std::printf("\n[ INFO] Waiting for stable initial \n");

        ros::Time t_check;
        t_check = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - t_check) < ros::Duration(20))
        {
            ros::spinOnce();
            rate.sleep();
        }
        std::printf("\n[ INFO] Init stable done \n");

        // init reference point
        global_ref_.latitude = current_global_.latitude;
        global_ref_.longitude = current_global_.longitude;
        global_ref_.altitude = current_global_.altitude;
        
        geometry_msgs::Point current_enu, goal_enu;
        for(int i = 0; i < 100; i++)
        {
            current_enu = WGS84ToENU(current_global_, global_ref_);
            x_off_[i] = current_pose_.pose.position.x - current_enu.x;
            y_off_[i] = current_pose_.pose.position.y - current_enu.y;
            z_off_[i] = current_pose_.pose.position.z - current_enu.z;

            ros::spinOnce();
            rate.sleep();
        }
        for(int i = 100; i > 0; --i)
        {
            x_offset_ = x_offset_ + x_off_[i]/100;
            y_offset_ = y_offset_ + y_off_[i]/100;
            z_offset_ = z_offset_ + z_off_[i]/100;
        }
        std::printf("\n[ INFO] Local position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
        std::printf("[ INFO] GPS position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);
        std::printf("\n[ DEBUG] global reference: %.8f, %.8f, %.3f\n", global_ref_.latitude, global_ref_.longitude, global_ref_.altitude);
        std::printf("[ DEBUG] offset: %.3f, %.3f, %.3f\n\n", x_offset_, y_offset_, z_offset_);
        inputTarget();
        if(input_type_ == true) // local setpoint
        {
            target_pose_ = targetTransfer(x_target_[0], y_target_[0], z_target_[0]);
        }
        else // global setpoint
        {
            goal_enu = WGS84ToENU(goalTransfer(lat_goal_[0], lon_goal_[0], alt_goal_[0]), global_ref_);
            target_pose_ = targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_);
        }

        // send a few setpoints before starting
        std::printf("[ INFO] Setting OFFBOARD stream \n");
        for(int i = 100; ros::ok() && i > 0; --i)
        {
            target_pose_.header.stamp = ros::Time::now();
            local_pose_pub_.publish(target_pose_);
            ros::spinOnce();
            rate.sleep();
        }
        std::printf("\n[ INFO] Set OFFBOARD stream done \n");
        std::printf("[ INFO] Waiting OFFBOARD switching \n");
        while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD"))
        {
            ros::spinOnce();
            rate.sleep();
        }
        std::printf("\n[ INFO] --------------- READY --------------- \n");
        takeOff(rate);

        int i = 0;
        while(ros::ok())
        {
            if(input_type_)
            {
                if(i < (target_num_-1))
                {
                    final_position_ = false;
                    ros::param::get("desired_velocity", vel_desired_);
                    vel_ = velLimit(vel_desired_, current_pose_, targetTransfer(x_target_[i], y_target_[i], z_target_[i]));
                }
                else
                {
                    final_position_ = true;
                    ros::param::get("desired_velocity", vel_desired_);
                    vel_ = velLimit(vel_desired_, current_pose_, targetTransfer(x_target_[target_num_-1], y_target_[target_num_-1], z_target_[target_num_-1]));
                }
                target_pose_ = targetTransfer(current_pose_.pose.position.x + vel_[0], current_pose_.pose.position.y + vel_[1], current_pose_.pose.position.z + vel_[2]);
                target_pose_.header.stamp = ros::Time::now();
                local_pose_pub_.publish(target_pose_);

                std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
                std::printf("Target local position: [%.3f, %.3f, %.3f]\n", x_target_[i], y_target_[i], z_target_[i]);

                distance_ = distanceMeasure(current_pose_, targetTransfer(x_target_[i], y_target_[i], z_target_[i]));
                std::printf("Distance to target: %.3f (m) \n", distance_);

                bool target_reached = checkPosition(check_error_, current_pose_, targetTransfer(x_target_[i], y_target_[i], z_target_[i]));
                std::printf("Target reached: %s\n", target_reached ? "true" : "false");

                if(target_reached && !final_position_)
                {
                    std::printf("\n[ INFO] Reached position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);   
                    std::printf("- Target position: [%.3f, %.3f, %.3f]\n", x_target_[i], y_target_[i], z_target_[i]);
                    std::printf("- Next target: [%.3f, %.3f, %.3f]\n", x_target_[i+1], y_target_[i+1], z_target_[i+1]);
                    
                    std::printf("- Hovering\n");
                    ros::param::get("hover_time",hover_time_);
                    hoverAt(hover_time_, targetTransfer(x_target_[i], y_target_[i], z_target_[i]), rate);
                    landingAt(targetTransfer(x_target_[i], y_target_[i], z_target_[i]), rate);
                    i+=1;
                }
                if(target_reached && final_position_)
                {
                    std::printf("\n[ INFO] Reached FINAL position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
                    std::printf("- Hovering\n");
                    ros::param::get("hover_time",hover_time_);
                    hoverAt(hover_time_, targetTransfer(x_target_[target_num_-1], y_target_[target_num_-1], z_target_[target_num_-1]), rate);
                    landingAtFinal(targetTransfer(x_target_[target_num_-1], y_target_[target_num_-1], z_target_[target_num_-1]), rate);
                    break;
                }
            }
            else // if(!input_type_)
            {
                if(i < (goal_num_-1))
                {
                    final_position_ = false;
                    goal_enu = WGS84ToENU(goalTransfer(lat_goal_[i], lon_goal_[i], alt_goal_[i]), global_ref_);
                }
                else
                {
                    final_position_ = true;
                    goal_enu = WGS84ToENU(goalTransfer(lat_goal_[goal_num_-1], lon_goal_[goal_num_-1], alt_goal_[goal_num_-1]), global_ref_);
                }
                ros::param::get("desired_velocity", vel_desired_);
                vel_ = velLimit(vel_desired_, current_pose_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_));

                target_pose_ = targetTransfer(current_pose_.pose.position.x + vel_[0], current_pose_.pose.position.y + vel_[1], current_pose_.pose.position.z + vel_[2]);
                target_pose_.header.stamp = ros::Time::now();
                local_pose_pub_.publish(target_pose_);

                std::printf("\nCurrent GPS position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);
                std::printf("Goal GPS position: [%.8f, %.8f, %.3f]\n", lat_goal_[i], lon_goal_[i], alt_goal_[i]);
            
                std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
                std::printf("Target local position: [%.3f, %.3f, %.3f]\n", targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.x, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.y, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.z);

                distance_ = distanceMeasure(current_pose_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_));
                std::printf("Distance to goal: %.3f (m) \n", distance_);

                bool target_reached = checkPosition(check_error_, current_pose_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_));
                std::printf("Target reached: %s\n", target_reached ? "true" : "false");

                if(target_reached && !final_position_)
                {
                    std::printf("\n[ INFO] Reached position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);   
                    std::printf("- Goal position: [%.8f, %.8f, %.3f]\n", lat_goal_[i], lon_goal_[i], alt_goal_[i]);
                    std::printf("- Next goal: [%.8f, %.8f, %.3f]\n", lat_goal_[i+1], lon_goal_[i+1], alt_goal_[i+1]);
                    std::printf("\n[ INFO] Local position: [%.3f, %.3f, %.3f]\n", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);   
                    std::printf("- Converted target: [%.3f, %.3f, %.3f]\n", targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.x, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.y, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_).pose.position.z);

                    std::printf("- Hovering\n");
                    ros::param::get("hover_time",hover_time_);
                    hoverAt(hover_time_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate);
                    landingAt(targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate);
                    i+=1;
                }
                if(target_reached && final_position_)
                {
                    std::printf("\n[ INFO] Reached FINAL position: [%.8f, %.8f, %.3f]\n", current_global_.latitude, current_global_.longitude, current_global_.altitude);
                    std::printf("- Hovering\n");
                    ros::param::get("hover_time",hover_time_);
                    hoverAt(hover_time_, targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate);
                    landingAtFinal(targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_), rate);
                    break;
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
    else
    {
        std::printf("\n[ WARN] Not avaible function - Please Relaunch\n");
    }

    return 0;
}