#include <offboard/offboard.h>
#include <offboard/logging.h>

/****** FUNCTIONS ******/

bool check_position(float error, geometry_msgs::PoseStamped current,
					geometry_msgs::PoseStamped target)
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

bool check_orientation(float error, geometry_msgs::PoseStamped current,
					geometry_msgs::PoseStamped target)
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

bool check_global(double lat, double lon, double alt,
				double lat0, double lon0, double alt0)
{
	if(((lat - 0.000001) < lat0) && (lat0 < (lat + 0.000001)) 
	 && ((lon - 0.000001) < lon0) && (lon0 < (lon + 0.000001))
	 && ((alt - 0.1) < alt0) && (alt0 < (alt + 0.1))
	)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void input_local_target()
{
	std::cout << "[ INFO] Input Local position(s)" << std::endl;
	std::cout << "Number of target(s): "; std::cin >> target_num;
	if(!in_x_pos.empty() || !in_y_pos.empty() || !in_z_pos.empty())
    {
        in_x_pos.clear();
		in_y_pos.clear();
		in_z_pos.clear();
    }
	for (int i = 0; i < target_num; i++)
	{
		std::cout << "Target (" << i+1 << ") position (in meter):" <<std::endl; 
		std::cout << "x (" << i+1 << "): "; std::cin >> in_x_pos[i];
		std::cout << "y (" << i+1 << "): "; std::cin >> in_y_pos[i];
		std::cout << "z (" << i+1 << "): "; std::cin >> in_z_pos[i];
		updates_local(i, in_x_pos[i], in_y_pos[i], in_z_pos[i]);
	}
	int count = 0;
	while (check_error < 0 || check_error > 1) 
	{
		std::cout << "Check offset value (0 < and < 1m): "; std::cin >> check_error;
		if (check_error < 0 || check_error > 1) 
		{
			count ++;
		}
		if (count = 3)
		{
			break;
		}
	}
	std::cout << "That error is out of range, set to default (0.1 m)" << std::endl;
	check_error = 0.1;
}

void input_global_target()
{
	std::cout << "Input GPS position(s)" << std::endl;
	std::cout << "Number of goal(s): "; std::cin >> goal_num;
	if(!in_latitude.empty() || !in_longitude.empty() || !in_altitude.empty())
    {
        in_latitude.clear();
		in_longitude.clear();
		in_altitude.clear();
    }
	for (int i = 0; i < goal_num; i++)
	{
		std::cout << "Goal ("<< i+1 <<") position:" << std::endl;
		std::cout << "Latitude " << i+1 << " (in degree): "; std::cin >> in_latitude[i];
		std::cout << "Longitude " << i+1 << " (in degree): "; std::cin >> in_longitude[i];
		std::cout << "Altitude " << i+1 << "  (in meter): "; std::cin >> in_altitude[i];
		updates_global(i, in_latitude[i], in_longitude[i], in_altitude[i]);
	}
	int count = 0;
	while (check_error < 0 || check_error > 1) 
	{
		std::cout << "Check offset value (0 < and < 1m): "; std::cin >> check_error;
		if (check_error < 0 || check_error > 1) 
		{
			count ++;
		}
		if (count = 3)
		{
			break;
		}
	}
	std::cout << "That error is out of range, set to default (0.3 m)" << std::endl;
	check_error = 0.3;
}

void input_target()
{
	char c;
	std::cout << "(1) Manual Input || Load Parameter (2) ? (1/2)\n"; std::cin >> c;
	if (c == '1')
	{
		std::cout << "Waypoint type: (3) Local || Global (4) ? (3/4)\n"; std::cin >> c;
		if (c == '3')
		{
			input_local_target();
			local_input = true;
		}
		else if (c == '4')
		{
			input_global_target();
			local_input = false;
		}
		else input_target();

	}
	else if (c == '2')
	{
		if(!in_x_pos.empty() || !in_y_pos.empty() || !in_z_pos.empty())
		{
			in_x_pos.clear();
			in_y_pos.clear();
			in_z_pos.clear();
		}
		if(!in_latitude.empty() || !in_longitude.empty() || !in_altitude.empty())
		{
			in_latitude.clear();
			in_longitude.clear();
			in_altitude.clear();
		}
		system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
    	std::cout << "[ INFO] Load parameters" << std::endl;
		ros::param::get("num_of_target", target_num);
		ros::param::get("x_pos", in_x_pos);
		ros::param::get("y_pos", in_y_pos);
		ros::param::get("z_pos", in_z_pos);
		ros::param::get("target_error", local_error);
		ros::param::get("num_of_goal", goal_num);
		ros::param::get("latitude", in_latitude);
		ros::param::get("longitude", in_longitude);
		ros::param::get("altitude", in_altitude);
		ros::param::get("goal_error", global_error);

		std::cout << "Waypoint type: (3) Local || (4) Global ? (3/4) \n"; std::cin >> c;
		if (c == '3')
		{
			local_input = true;
			check_error = local_error;
			target_num = target_num;
			for (int i = 0; i < target_num; i++)
			{
				std::cout << "Target (" << i+1 << "): [" << in_x_pos[i] << ", "
														 << in_y_pos[i] << ", "
														 << in_z_pos[i] << "]\n";
				updates_local(i+1, in_x_pos[i], in_y_pos[i], in_z_pos[i]);
			}
			std::cout << "Check offset value: " << check_error << " (m)\n";
		}
		else if (c == '4')
		{
			local_input = false;
			check_error = global_error;
			goal_num = goal_num;
			for (int i = 0; i < goal_num; i++)
			{
				updates_global(i+1, in_latitude[i], in_longitude[i], in_altitude[i]);
				std::cout << "Goal (" << i+1 << "): [" << in_latitude[i] << ", "
													   << in_longitude[i] << ", "
													   << in_altitude[i] << "]" << std::endl;
			}
			std::cout << "Check offset value: " << check_error << " (m)\n";
		}
		else input_target();
	}
	else 
	{
		input_target();
	}
	
}

double degree(double rad)
{
	return (rad*180)/PI;
}

double radian(double deg)
{
	return (deg*PI)/180;
}

double measureGPS(double lat1, double lon1, double alt1, 
				  double lat2, double lon2, double alt2)
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

double distanceLocal(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
	double xc = current.pose.position.x;
	double yc = current.pose.position.y;
	double zc = current.pose.position.z;
	double xt = target.pose.position.x;
	double yt = target.pose.position.y;
	double zt = target.pose.position.z;

	return sqrt((xt-xc)*(xt-xc) + (yt-yc)*(yt-yc) + (zt-zc)*(zt-zc));
}

void velociyGenerate(geometry_msgs::PoseStamped current, 
					 geometry_msgs::PoseStamped target, 
					 double* vx, double* vy, double* vz)
{
	double xt = target.pose.position.x;	
	double yt = target.pose.position.y;
	double zt = target.pose.position.z;

	double xc = current.pose.position.x;	
	double yc = current.pose.position.y;
	double zc = current.pose.position.z;
	
	double dx = xt - xc; 
	double dy = yt - yc; 
	double dz = zt - zc; 

	double d = sqrt(dx*dx + dy*dx + dz*dz);
	double v;
	system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
	ros::param::get("v_desired", v);
	*vx = (dx/d) * v;
	*vy = (dy/d) * v;
	*vz = (dz/d) * v;
}

// void get_GPS(int gps_lat, int gps_lon, int gps_alt, double* lat, double* lon, double* alt)
// {
// 	*lat = double(gps_lat)/10000000;
//     *lon = double(gps_lon)/10000000;
//     *alt = double(gps_alt)/1000;
// }

geometry_msgs::Point WGS84ToECEF(double lat, double lon, double alt)
{
    geometry_msgs::Point ecef;
    double lambda = radian(lat);
    double phi = radian(lon);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    ecef.x = (alt + N) * cos_lambda * cos_phi;
    ecef.y = (alt + N) * cos_lambda * sin_phi;
    ecef.z = (alt + (1 - e_sq) * N) * sin_lambda;

    return ecef;
}

geographic_msgs::GeoPoint ECEFToWGS84(double x, double y, double z)
{
    geographic_msgs::GeoPoint wgs84;
    double eps = e_sq / (1.0 - e_sq);
    double p = sqrt(x * x + y * y);
    double q = atan2((z * a), (p * b));
    double sin_q = sin(q);
    double cos_q = cos(q);
    double sin_q_3 = sin_q * sin_q * sin_q;
    double cos_q_3 = cos_q * cos_q * cos_q;
    double phi = atan2((z + eps * b * sin_q_3), (p - e_sq * a * cos_q_3));
    double lambda = atan2(y, x);
    double v = a / sqrt(1.0 - e_sq * sin(phi) * sin(phi));
    
    wgs84.altitude = (p / cos(phi)) - v;

    wgs84.latitude = degree(phi);
    wgs84.longitude = degree(lambda);

    return wgs84;
}

geometry_msgs::Point ECEFToENU(double x, double y, double z,
                               double lat0, double lon0, double alt0)
{
    geometry_msgs::Point enu;
    double lambda = radian(lat0);
    double phi = radian(lon0);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x0 = (alt0 + N) * cos_lambda * cos_phi;
    double y0 = (alt0 + N) * cos_lambda * sin_phi;
    double z0 = (alt0 + (1 - e_sq) * N) * sin_lambda;

    double xd, yd, zd;
    xd = x - x0;
    yd = y - y0;
    zd = z - z0;

    enu.x = -sin_phi * xd + cos_phi * yd;
    enu.y = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    enu.z = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

    return enu;
}

geometry_msgs::Point ENUToECEF(double xEast, double yNorth, double zUp,
                                double lat0, double lon0, double alt0)
{
    geometry_msgs::Point ecef;
    double lambda = radian(lat0);
    double phi = radian(lon0);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x0 = (alt0 + N) * cos_lambda * cos_phi;
    double y0 = (alt0 + N) * cos_lambda * sin_phi;
    double z0 = (alt0 + (1 - e_sq) * N) * sin_lambda;

    double xd = -sin_phi * xEast - cos_phi * sin_lambda * yNorth + cos_lambda * cos_phi * zUp;
    double yd = cos_phi * xEast - sin_lambda * sin_phi * yNorth + cos_lambda * sin_phi * zUp;
    double zd = cos_lambda * yNorth + sin_lambda * zUp;

    ecef.x = xd + x0;
    ecef.y = yd + y0;
    ecef.z = zd + z0;

    return ecef;
}

geometry_msgs::Point WGS84ToENU(double lat, double lon, double alt,
                                double lat0, double lon0, double alt0)
{
    geometry_msgs::Point ecef = WGS84ToECEF(lat, lon, alt);
    geometry_msgs::Point enu = ECEFToENU(ecef.x, ecef.y, ecef.z,
                                        lat0, lon0, alt0);
    return enu;
}

geographic_msgs::GeoPoint ENUToWGS84(double xEast, double yNorth, double zUp,
                                      double lat0, double lon0, double alt0)
{
    geometry_msgs::Point ecef = ENUToECEF(xEast, yNorth, zUp,
                                          lat0, lon0, alt0);
    geographic_msgs::GeoPoint wgs84 = ECEFToWGS84(ecef.x, ecef.y, ecef.z);

    return wgs84;
}
