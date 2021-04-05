#include <offboard/offboard.h>

OffboardControl::OffboardControl()
{
	// constructor
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}
void localPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = *msg;
}
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    global_position_ = *msg;
    global_position_received = true;
}
void gpsPosition_cb(const mavros_msgs::GPSRAW::ConstPtr& msg)
{
    gps_position_ = *msg;
    gps_position_received = true;
}

void OffboardControl::position_pub(ros::NodeHandle nh)
{
	nh_ = nh;
	state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	local_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, localPose_cb);
	global_pos_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, globalPosition_cb);
	gps_pos_sub_ = nh_.subscribe<mavros_msgs::GPSRAW>("mavros/gpsstatus/gps1/raw", 10, gpsPosition_cb);

	local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
	
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    std::cout << "[ INFO] Waiting for FCU connection...\n";
    while(ros::ok() && !current_state_.connected)
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
	std::cout << "[ INFO] Waiting for stable initial... \n";
    t_check_ = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - t_check_) < ros::Duration(20))
    {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Stable initial done \n";

	// init reference point
    ref_.latitude = global_position_.latitude;
    ref_.longitude = global_position_.longitude;
    ref_.altitude = global_position_.altitude;
	for(int i = 100; ros::ok() && i > 0; --i)
    {
        enu_c_ = WGS84ToENU(global_position_, ref_);
        x_off_[i] = current_pose_.pose.position.x - enu_c_.x;
        y_off_[i] = current_pose_.pose.position.y - enu_c_.y;
        z_off_[i] = current_pose_.pose.position.z - enu_c_.z;

        ros::spinOnce();
        rate.sleep();
    }
    for(int i = 100; i > 0; --i)
    {
        x_offset_ = x_offset_ + x_off_[i]/100;
        y_offset_ = y_offset_ + y_off_[i]/100;
        z_offset_ = z_offset_ + z_off_[i]/100;
    }

	input_target();
    if (local_input_ == true) // local setpoint
    {
        target_pose_.pose.position.x = in_x_pos_[0];
        target_pose_.pose.position.y = in_y_pos_[0];
        target_pose_.pose.position.z = in_z_pos_[0];
    }
    else // global setpoint
    {
        enu_g_ = WGS84ToENU(goalTransfer(in_latitude_[0], in_longitude_[0], in_altitude_[0]), ref_);
        target_pose_.pose.position.x = enu_g_.x + x_offset_;
        target_pose_.pose.position.y = enu_g_.y + y_offset_;
        target_pose_.pose.position.z = enu_g_.z + z_offset_;
    }

    // send a few setpoints before starting
    std::cout << "[ INFO] Setting OFFBOARD stream...\n";
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        target_pose_.header.stamp = ros::Time::now();
        local_pos_pub_.publish(target_pose_);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Set OFFBOARD stream done \n";

    std::cout << "[ INFO] Waiting... \n";
    while (ros::ok() && !current_state_.armed)
    {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] ---------- READY ---------- \n";
	position_control(local_input_, rate);
}

void OffboardControl::position_control(bool local_input, ros::Rate rate)
{
	int i = 0;
	if (local_input_)
	{
		while (ros::ok())
        {
            if (i < (target_num_ -1))
            {
                final_check_ = false;
                target_pose_.pose.position.x = in_x_pos_[i];
                target_pose_.pose.position.y = in_y_pos_[i];
                target_pose_.pose.position.z = in_z_pos_[i];
            
                target_pose_.header.stamp = ros::Time::now();
                local_pos_pub_.publish(target_pose_);

        		ros::spinOnce();
                rate.sleep();
            }
            else
            {
                final_check_ = true;
                target_pose_.pose.position.x = in_x_pos_[target_num_ - 1];
                target_pose_.pose.position.y = in_y_pos_[target_num_ - 1];
                target_pose_.pose.position.z = in_z_pos_[target_num_ - 1];
            
                target_pose_.header.stamp = ros::Time::now();
                local_pos_pub_.publish(target_pose_);

        		ros::spinOnce();
                rate.sleep();
            }
            std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", 
                         current_pose_.pose.position.x, 
                         current_pose_.pose.position.y, 
                         current_pose_.pose.position.z);
            std::printf("Target local position: [%.3f, %.3f, %.3f]\n", 
                        in_x_pos_[i], in_y_pos_[i], in_z_pos_[i]);
            distance_ = distanceLocal(current_pose_, target_pose_);
            std::printf("Distance to target: %.3f m \n", distance_);

			system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
			ros::param::get("hover_time", t_hover_);

            bool check = check_position(check_error_, current_pose_,target_pose_);
            std::cout << "\n" << check << std::endl;
            if(check && !final_check_)
            {
                t_check_ = ros::Time::now();
                std::printf("[ INFO] Reached position: [%.3f, %.3f, %.3f]\n", 
                                current_pose_.pose.position.x, 
                                current_pose_.pose.position.y, 
                                current_pose_.pose.position.z);   
                std::printf("[ INFO] Next local position: [%.3f, %.3f, %.3f\n", in_x_pos_[i+1], in_y_pos_[i+1], in_z_pos_[i+1]);
                std::cout << "[ INFO] Hover at checkpoint \n";

                while ((ros::Time::now() - t_check_) < ros::Duration(t_hover_))
                {
                    local_pos_pub_.publish(target_pose_);

                    ros::spinOnce();
    		        rate.sleep();
                }

                i = i + 1;
                ros::spinOnce();
    		    rate.sleep();
    		}
            else if (check && final_check_)
            {
                t_check_ = ros::Time::now();
                std::printf("[ INFO] Reached FINAL position: [%.3f, %.3f, %.3f]\n", 
                                current_pose_.pose.position.x, 
                                current_pose_.pose.position.y, 
                                current_pose_.pose.position.z);
                std::printf("[ INFO] Ready to LANDING \n");

                while ((ros::Time::now() - t_check_) < ros::Duration(t_hover_))
                {
                    local_pos_pub_.publish(target_pose_);

                    ros::spinOnce();
    		        rate.sleep();
                }

    			set_mode_.request.custom_mode = "AUTO.LAND";
        	    if( set_mode_client_.call(set_mode_) && set_mode_.response.mode_sent)
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
	}
	else // global setpoints
    {
        while (ros::ok())
        {
            if (i < (goal_num_ - 1))
            {
                final_check_ = false;
                enu_g_ = WGS84ToENU(goalTransfer(in_latitude_[i], in_longitude_[i], in_altitude_[i]), ref_);
                target_pose_.pose.position.x = enu_g_.x + x_offset_;
                target_pose_.pose.position.y = enu_g_.y + y_offset_;
                target_pose_.pose.position.z = enu_g_.z + z_offset_;
                
                target_pose_.header.stamp = ros::Time::now();
                local_pos_pub_.publish(target_pose_);

                ros::spinOnce();
                rate.sleep();
            }
            else
            {
                final_check_ = true;
                enu_g_ = WGS84ToENU(goalTransfer(in_latitude_[goal_num_ -1], in_longitude_[goal_num_ -1], in_altitude_[goal_num_ -1]), ref_);
                target_pose_.pose.position.x = enu_g_.x + x_offset_;
                target_pose_.pose.position.y = enu_g_.y + y_offset_;
                target_pose_.pose.position.z = enu_g_.z + z_offset_;
                
                target_pose_.header.stamp = ros::Time::now();
                local_pos_pub_.publish(target_pose_);

                ros::spinOnce();
                rate.sleep();
            }
            std::printf("\nCurrent GPS position: [%.8f, %.8f, %.3f]\n", 
                        global_position_.latitude, 
                        global_position_.longitude, 
                        global_position_.altitude);
            std::printf("Goal GPS position: [%.8f, %.8f, %.3f]\n", 
                        in_latitude_[i], in_longitude_[i], in_altitude_[i]);
           
            std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", 
                         current_pose_.pose.position.x, 
                         current_pose_.pose.position.y, 
                         current_pose_.pose.position.z);
            std::printf("Target local position: [%.3f, %.3f, %.3f]\n", 
                                target_pose_.pose.position.x, 
                                target_pose_.pose.position.y,
                                target_pose_.pose.position.z);
            distance_ = distanceLocal(current_pose_, target_pose_);
            std::printf("Distance to target: %.3f m \n", distance_);

			system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
			ros::param::get("hover_time", t_hover_);

            bool check = check_position(check_error_, current_pose_, target_pose_);

            std::cout << "\n" << check << std::endl;
            if (check && !final_check_)
            {
                t_check_ = ros::Time::now();
                std::printf("[ INFO] Reached position: [%.8f, %.8f, %.3f]\n", 
                                global_position_.latitude, 
                                global_position_.longitude, 
                                global_position_.altitude);
                std::printf("[ INFO] Next GPS position: [%.8f, %.8f, %.3f]\n", in_latitude_[i], in_latitude_[i], in_altitude_[i]);
                std::printf("[ INFO] Reached position: [%.3f, %.3f, %.3f]\n", 
                                current_pose_.pose.position.x, 
                                current_pose_.pose.position.y, 
                                current_pose_.pose.position.z);   
                std::printf("[ INFO] Target local position: [%.3f, %.3f, %.3f]\n", 
                                target_pose_.pose.position.x, 
                                target_pose_.pose.position.y,
                                target_pose_.pose.position.z);
                std::cout << "[ INFO] Hover at checkpoint \n";

                while ((ros::Time::now() - t_check_) < ros::Duration(t_hover_))
                {
                    local_pos_pub_.publish(target_pose_);

                    ros::spinOnce();
    		        rate.sleep();
                }

                i = i + 1;
                ros::spinOnce();
                rate.sleep();
            }
            else if (check && final_check_)
            {
                t_check_ = ros::Time::now();
                std::printf("[ INFO] Reached FINAL position: [%.8f, %.8f, %.3f]\n", 
                                global_position_.latitude, 
                                global_position_.longitude, 
                                global_position_.altitude);
                std::printf("[ INFO] Reached local position: [%.3f, %.3f, %.3f]\n", 
                                current_pose_.pose.position.x, 
                                current_pose_.pose.position.y, 
                                current_pose_.pose.position.z);
                std::printf("[ INFO] Target local position: [%.3f, %.3f, %.3f]\n", 
                                target_pose_.pose.position.x, 
                                target_pose_.pose.position.y,
                                target_pose_.pose.position.z);
                std::printf("[ INFO] Ready to LANDING \n");

                while ((ros::Time::now() - t_check_) < ros::Duration(t_hover_))
                {
                    local_pos_pub_.publish(target_pose_);

                    ros::spinOnce();
    		        rate.sleep();
                }

                set_mode_.request.custom_mode = "AUTO.LAND";
                if( set_mode_client_.call(set_mode_) && set_mode_.response.mode_sent)
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
    }
}

bool OffboardControl::check_position(float error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
	double xt = target.pose.position.x;
	double yt = target.pose.position.y;
	double zt = target.pose.position.z;
	double xc = current.pose.position.x;
	double yc = current.pose.position.y;
	double zc = current.pose.position.z;

	if(((xt - error) < xc) && (xc < (xt + error)) 
	&& ((yt - error) < yc) && (yc < (yt + error))
	&& ((zt - error) < zc) && (zc < (zt + error)))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

bool OffboardControl::check_orientation(float error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
	double currentx = current.pose.orientation.x;
	double currenty = current.pose.orientation.y;
	double currentz = current.pose.orientation.z;
	double currentw = current.pose.orientation.w;
	double target_x = target.pose.orientation.x;
	double target_y = target.pose.orientation.y;
	double target_z = target.pose.orientation.z;
	double target_w = target.pose.orientation.w;

	// tf Quaternion to RPY
	tf::Quaternion qc(currentx, currenty, currentz, currentw);
	tf::Matrix3x3 mc(qc);
	double rc, pc, yc;
	mc.getRPY(rc, pc, yc);

	tf::Quaternion qt(target_x, target_y, target_z, target_w);
	tf::Matrix3x3 mt(qt);
	double rt, pt, yt;
	mt.getRPY(rt, pt, yt);

	// check
	if((((degree(rt)-1)<(degree(rc)))&&(degree(rc)<(degree(rt)+1)))
	 &&(((degree(pt)-1)<(degree(pc)))&&(degree(pc)<(degree(pt)+1)))
	 &&(((degree(yt)-1)<(degree(yc)))&&(degree(yc)<(degree(yt)+1)))) 
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void OffboardControl::input_local_target()
{
	std::cout << "[ INFO] Input Local position(s)" << std::endl;
	std::cout << "Number of target(s): "; std::cin >> target_num_;
	if(!in_x_pos_.empty() || !in_y_pos_.empty() || !in_z_pos_.empty())
    {
        in_x_pos_.clear();
		in_y_pos_.clear();
		in_z_pos_.clear();
    }
	for (int i = 0; i < target_num_; i++)
	{
		std::cout << "Target (" << i+1 << ") position (in meter):" <<std::endl; 
		std::cout << "x (" << i+1 << "): "; std::cin >> in_x_pos_[i];
		std::cout << "y (" << i+1 << "): "; std::cin >> in_y_pos_[i];
		std::cout << "z (" << i+1 << "): "; std::cin >> in_z_pos_[i];
	}
	int count = 0;
	while (check_error_ < 0 || check_error_ > 1) 
	{
		std::cout << "Check offset value (0 < and < 1m): "; std::cin >> check_error_;
		if (check_error_ < 0 || check_error_ > 1) 
		{
			count ++;
		}
		if (count = 5)
		{
			break;
		}
	}
	std::cout << "That error is out of range, set to 0.1 m" << std::endl;
	check_error_ = 0.1;
}

void OffboardControl::input_global_target()
{
	std::cout << "[ INFO] Input GPS position(s)" << std::endl;
	std::cout << "Number of goal(s): "; std::cin >> goal_num_;
	if(!in_latitude_.empty() || !in_longitude_.empty() || !in_altitude_.empty())
    {
        in_latitude_.clear();
		in_longitude_.clear();
		in_altitude_.clear();
    }
	for (int i = 0; i < goal_num_; i++)
	{
		std::cout << "Goal ("<< i+1 <<") position:" << std::endl;
		std::cout << "Latitude " << i+1 << " (in degree): "; std::cin >> in_latitude_[i];
		std::cout << "Longitude " << i+1 << " (in degree): "; std::cin >> in_longitude_[i];
		std::cout << "Altitude " << i+1 << "  (in meter): "; std::cin >> in_altitude_[i];
	}
	int count = 0;
	while (check_error_ < 0 || check_error_ > 1) 
	{
		std::cout << "Check offset value (0 < and < 1m): "; std::cin >> check_error_;
		if (check_error_ < 0 || check_error_ > 1) 
		{
			count ++;
		}
		if (count = 5)
		{
			break;
		}
	}
	std::cout << "That error is out of range, set to 0.3 m" << std::endl;
	check_error_ = 0.3;
}

void OffboardControl::input_target()
{
	char c;
	std::cout << "[ INFO] (1) Manual Input || Load Parameter (2) ? (1/2)\n"; std::cin >> c;
	if (c == '1')
	{
		std::cout << "Waypoint type: (3) Local || Global (4) ? (3/4)\n"; std::cin >> c;
		if (c == '3')
		{
			input_local_target();
			local_input_ = true;
		}
		else if (c == '4')
		{
			input_global_target();
			local_input_ = false;
		}
		else input_target();

	}
	else if (c == '2')
	{
		if(!in_x_pos_.empty() || !in_y_pos_.empty() || !in_z_pos_.empty())
		{
			in_x_pos_.clear();
			in_y_pos_.clear();
			in_z_pos_.clear();
		}
		if(!in_latitude_.empty() || !in_longitude_.empty() || !in_altitude_.empty())
		{
			in_latitude_.clear();
			in_longitude_.clear();
			in_altitude_.clear();
		}
		system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
    	std::cout << "[ INFO] Load parameters" << std::endl;
		ros::param::get("num_of_target", target_num_);
		ros::param::get("x_pos", in_x_pos_);
		ros::param::get("y_pos", in_y_pos_);
		ros::param::get("z_pos", in_z_pos_);
		ros::param::get("target_error", local_error_);
		ros::param::get("num_of_goal", goal_num_);
		ros::param::get("latitude", in_latitude_);
		ros::param::get("longitude", in_longitude_);
		ros::param::get("altitude", in_altitude_);
		ros::param::get("goal_error", global_error_);

		std::cout << "Waypoint type: (3) Local || (4) Global ? (3/4) \n"; std::cin >> c;
		if (c == '3')
		{
			local_input_ = true;
			check_error_ = local_error_;
			for (int i = 0; i < target_num_; i++)
			{
				std::cout << "Target (" << i+1 << "): [" << in_x_pos_[i] << ", "
														 << in_y_pos_[i] << ", "
														 << in_z_pos_[i] << "]\n";
			}
			std::cout << "Check offset value: " << check_error_ << " (m)\n";
		}
		else if (c == '4')
		{
			local_input_ = false;
			check_error_ = global_error_;
			for (int i = 0; i < goal_num_; i++)
			{
				std::cout << "Goal (" << i+1 << "): [" << in_latitude_[i] << ", "
													   << in_longitude_[i] << ", "
													   << in_altitude_[i] << "]" << std::endl;
			}
			std::cout << "Check offset value: " << check_error_ << " (m)\n";
		}
		else input_target();
	}
	else 
	{
		input_target();
	}
	
}

double OffboardControl::degree(double rad)
{
	return (rad*180)/PI;
}

double OffboardControl::radian(double deg)
{
	return (deg*PI)/180;
}

double OffboardControl::measureGPS(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2)
{
	double flat, plus, Distance;
	lat1 = radian(lat1); lon1 = radian(lon1);
	lat2 = radian(lat2); lon2 = radian(lon2);
	flat = 2*eR*asin(sqrt(sin((lat2-lat1)/2)*sin((lat2-lat1)/2)
	       +cos(lat1)*cos(lat2)*sin((lon2-lon1)/2)*sin((lon2-lon1)/2)))*1000; //m
	alt1 = abs(alt1);
	alt2 = abs(alt2);
	if (alt1 == alt2)
	{
		Distance = flat;
	}
	else
	{
		if 	(alt1 > alt2)
		{
			plus = flat/((alt1/alt2)-1);
			Distance = sqrt((flat+plus)*(flat+plus) + alt1*alt1) 
					   - sqrt(plus*plus + alt2*alt2);
		}
		else
		{
			plus = flat/((alt2/alt1)-1);
			Distance = sqrt((flat+plus)*(flat+plus) + alt2*alt2) 
					   - sqrt(plus*plus + alt1*alt1);
		}
	}
	return Distance;
}

double OffboardControl::distanceLocal(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
	double xc = current.pose.position.x;
	double yc = current.pose.position.y;
	double zc = current.pose.position.z;
	double xt = target.pose.position.x;
	double yt = target.pose.position.y;
	double zt = target.pose.position.z;

	return sqrt((xt-xc)*(xt-xc) + (yt-yc)*(yt-yc) + (zt-zc)*(zt-zc));
}

geometry_msgs::Point OffboardControl::WGS84ToECEF(sensor_msgs::NavSatFix wgs84)
{
    geometry_msgs::Point ecef;
    double lambda = radian(wgs84.latitude);
    double phi = radian(wgs84.longitude);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    ecef.x = (wgs84.altitude + N) * cos_lambda * cos_phi;
    ecef.y = (wgs84.altitude + N) * cos_lambda * sin_phi;
    ecef.z = (wgs84.altitude + (1 - e_sq) * N) * sin_lambda;

    return ecef;
}

geographic_msgs::GeoPoint OffboardControl::ECEFToWGS84(geometry_msgs::Point ecef)
{
    geographic_msgs::GeoPoint wgs84;
    double eps = e_sq / (1.0 - e_sq);
    double p = sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
    double q = atan2((ecef.z * a), (p * b));
    double sin_q = sin(q);
    double cos_q = cos(q);
    double sin_q_3 = sin_q * sin_q * sin_q;
    double cos_q_3 = cos_q * cos_q * cos_q;
    double phi = atan2((ecef.z + eps * b * sin_q_3), (p - e_sq * a * cos_q_3));
    double lambda = atan2(ecef.y, ecef.x);
    double v = a / sqrt(1.0 - e_sq * sin(phi) * sin(phi));
    
    wgs84.altitude = (p / cos(phi)) - v;

    wgs84.latitude = degree(phi);
    wgs84.longitude = degree(lambda);

    return wgs84;
}

geometry_msgs::Point OffboardControl::ECEFToENU(geometry_msgs::Point ecef, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point enu;
    double lambda = radian(ref.latitude);
    double phi = radian(ref.longitude);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x0 = (ref.altitude + N) * cos_lambda * cos_phi;
    double y0 = (ref.altitude + N) * cos_lambda * sin_phi;
    double z0 = (ref.altitude + (1 - e_sq) * N) * sin_lambda;

    double xd, yd, zd;
    xd = ecef.x - x0;
    yd = ecef.y - y0;
    zd = ecef.z - z0;

    enu.x = -sin_phi * xd + cos_phi * yd;
    enu.y = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    enu.z = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

    return enu;
}

geometry_msgs::Point OffboardControl::ENUToECEF(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point ecef;
    double lambda = radian(ref.latitude);
    double phi = radian(ref.longitude);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x0 = (ref.altitude + N) * cos_lambda * cos_phi;
    double y0 = (ref.altitude + N) * cos_lambda * sin_phi;
    double z0 = (ref.altitude + (1 - e_sq) * N) * sin_lambda;

    double xd = -sin_phi * enu.x - cos_phi * sin_lambda * enu.y + cos_lambda * cos_phi * enu.z;
    double yd = cos_phi * enu.x - sin_lambda * sin_phi * enu.y + cos_lambda * sin_phi * enu.z;
    double zd = cos_lambda * enu.y + sin_lambda * enu.z;

    ecef.x = xd + x0;
    ecef.y = yd + y0;
    ecef.z = zd + z0;

    return ecef;
}

geometry_msgs::Point OffboardControl::WGS84ToENU(sensor_msgs::NavSatFix wgs84, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point ecef = WGS84ToECEF(wgs84);
    geometry_msgs::Point enu = ECEFToENU(ecef, ref);
    return enu;
}

geographic_msgs::GeoPoint OffboardControl::ENUToWGS84(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point ecef = ENUToECEF(enu, ref);
    geographic_msgs::GeoPoint wgs84 = ECEFToWGS84(ecef);

    return wgs84;
}

OffboardControl::~OffboardControl()
{
	// destructor
}