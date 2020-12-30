/******* ros *******/
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/BatteryState.h>
/******* local position *******/
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
/******* global position *******/
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPoint.h>
/******* tranformation *******/
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
/******* c++ *******/
// #include <iostream>
// #include <cmath>
// #include <cstdio>
// #include <fstream>
// #include <iomanip>
#include "offboard/logs.h"


/****** DEFINE CONSTANTS ******/
const double PI  =3.141592653589793238463;
const double eR  =6378.137; // earth radius in km

const double a = 6378137.0;         // WGS-84 Earth semimajor axis (m)
const double b = 6356752.314245;     // Derived Earth semiminor axis (m)
const double f = (a - b) / a;           // Ellipsoid Flatness
const double f_inv = 1.0 / f;       // Inverse flattening

//const double f_inv = 298.257223563; // WGS-84 Flattening Factor of the Earth 
//const double b = a - a / f_inv;
//const double f = 1.0 / f_inv;

const double a_sq = a * a;
const double b_sq = b * b;
const double e_sq = f * (2 - f);    // Square of Eccentricity

/****** DEFINE FUNCTIONS ******/
/********************************************************************************/
/***** check_position: check when drone reached the target positions       ******/
/***** input: x_target, y_target, ztarget, x_current, y_current, z_current ******/
/***** return: true or false                                               ******/          
/********************************************************************************/
bool check_position(double, double, double,
				    double, double, double);

/*******************************************************************************************/
/***** check_orientation: check when drone reached the target orientations            ******/
/***** input: (quaternion target) xt, yt, zt, wt, (quaternion current) xc, yc, zc, wc ******/
/***** return: true or false                                                          ******/
/*******************************************************************************************/
bool check_orientation(double, double, double, double,
				       double, double, double, double);

/**************************************************************************/
/***** check_global: check when drone reached the GPS goal positions ******/
/***** input: (target) lat, lon, alt, (current) lat0, lon0, alt0     ******/
/***** return: true or false                                         ******/       
/**************************************************************************/
bool check_global(double, double, double,
				  double, double, double);

/***************************************************************************/
/***** input_local_target: input the local coodinate (x, y, z) points ******/
/***** and (maybe) input the yaw rotation at each point               ******/
/***************************************************************************/
void input_local_target(void);

/****************************************************************************************/
/***** input_global_target: input the GPS [latitude, longitude, altitude] point(s) ******/
/****************************************************************************************/
void input_global_target(void);

/*******************************************************************/
/***** input_target: choose the local or global input targets ******/
/*******************************************************************/
void input_target(void);

/********************************************************/
/***** degree: convert angle from radian to degree ******/
/***** input: angle in radian                      ******/
/***** return: angle in degree                     ******/
/********************************************************/
double degree(double);

/********************************************************/
/***** radian: convert angle from degree to radian ******/
/***** input: angle in degree                      ******/
/***** return: angle in radian                     ******/
/********************************************************/
double radian(double);

/*********************************************************************************************/
/***** measureGPS: measure the distance between 2 GPS points that use haversine formula ******/
/***** input: (GPS1) lat1, lon1, alt1, (GPS2) lat2, lon2, alt2                          ******/
/***** return: distance in meters                                                       ******/
/*********************************************************************************************/
double measureGPS(double, double, double, double, double, double);

/* WGS84ToECEF: Converts the WGS-84 Geodetic point (latitude, longitude, altitude) **
** to Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z)                      **/
geometry_msgs::Point WGS84ToECEF(double, double, double);

/* ECEFToWGS84: Converts the Earth-Centered Earth-Fixed coordinates (x, y, z) to   **
** WGS-84 Geodetic point (latitude, longitude, altitude)                           **/
geographic_msgs::GeoPoint ECEFToWGS84(double, double, double);

/* ECEFToENU: Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) **
** to East-North-Up coordinates in a Local Tangent Plane that is centered at the   **
** (WGS-84) Geodetic point (lat0, lon0, alt0)                                      **/
geometry_msgs::Point ECEFToENU(double, double, double, double, double, double);

/* ENUToECEF: Converts East-North-Up coordinates (xEast, yNorth, zUp) in a Local   **
** Tangent Plane that is centered at  (WGS-84) Geodetic point (lat0, lon0, alt0)   **
** to the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z)                  **/
geometry_msgs::Point ENUToECEF(double, double, double, double, double, double);

/* WGS84ToENU: Converts the geodetic WGS-84 coordinated (lat, lon, alt) to         **
** East-North-Up coordinates in a Local Tangent Plane that is centered at the      **
** (WGS-84) Geodetic point (lat0, lon0, alt0)                                      **/
geometry_msgs::Point WGS84ToENU(double, double, double, double, double, double);

/* ENUToWGS84: Converts the East-North-Up coordinates in a Local Tangent Plane to  **
** geodetic WGS-84 coordinated (lat, lon, alt) that is centered at the (WGS-84)    **
** Geodetic point (lat0, lon0, alt0)                                               **/
geographic_msgs::GeoPoint ENUToWGS84(double, double, double, double, double, double);

// void creates(std::string, double, double, double, 
//             double, double, double);
// void updates(std::string, double, double, double, 
//                    double, double, double);
// void updates_check(int, double, double, double, 
//                    double, double, double);
// void updates_local(int, double, double, double); 
// void updates_global(int, double, double, double);

/****** DECLARE VARIANTS ******/
mavros_msgs::State current_state;
mavros_msgs::SetMode set_mode;

geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped target_pose;

bool global_position_received = false;
sensor_msgs::NavSatFix global_position;
geographic_msgs::GeoPoseStamped goal_position;

sensor_msgs::BatteryState current_batt;

tf::Quaternion q; // quaternion to transform to RPY

int target_num; // number of local setpoints
double target_pos[10][7]; // local setpoints list
double roll, pitch, yaw; // roll, pitch, yaw angle
double r, p, y; // roll, pitch, yaw use in transform

int goal_num; // number of global setpoints
double goal_pos[10][3]; // global setpoints list
double latitude, longitude, altitude, distance;

bool input_type = true; // true == input local || false == input global setpoints
bool final_check = false; // true == reached final point || false == NOT final point
float batt_percent; // baterry capacity

geometry_msgs::Point enu_goal, enu_curr; //Local ENU points: converted from GPS goal and current
geographic_msgs::GeoPoint wgs84_target, wgs84_curr; //Global WGS84 point: convert from ENU target and current
geographic_msgs::GeoPoint refpoint; //Reference point to convert ECEF to ENU and vice versa

/***** callback functions *****/
// state callback 
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// local pose callback
void localPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

// global position callback
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
    global_position = *msg;
    global_position_received = true;
}

// battery status callback
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg) 
{
    current_batt = *msg;
}

/***** executed functions *****/
bool check_position(double target_x, double target_y, double target_z,
					double currentx, double currenty, double currentz)
{
	bool reached;
	if(((target_x - 0.1) < currentx)
	&& (currentx < (target_x + 0.1)) 
	&& ((target_y - 0.1) < currenty)
	&& (currenty < (target_y + 0.1))
	&& ((target_z - 0.1) < currentz)
	&& (currentz < (target_z + 0.1)))
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}

bool check_orientation(double target_x, double target_y, double target_z, double target_w,
					   double currentx, double currenty, double currentz, double currentw)
{
	bool reached;
	// tf Quaternion to RPY
	tf::Quaternion qc(currentx,
					  currenty,
					  currentz,
					  currentw);
	tf::Matrix3x3 mc(qc);
	double rc, pc, yc;
	mc.getRPY(rc, pc, yc);

	tf::Quaternion qt(target_x,
					  target_y,
					  target_z,
					  target_w);
	tf::Matrix3x3 mt(qt);
	double rt, pt, yt;
	mt.getRPY(rt, pt, yt);
	// check
	if((((degree(rt)-1)<(degree(rc)))&&(degree(rc)<(degree(rt)+1)))
	 &&(((degree(pt)-1)<(degree(pc)))&&(degree(pc)<(degree(pt)+1)))
	 &&(((degree(yt)-1)<(degree(yc)))&&(degree(yc)<(degree(yt)+1)))) 
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}

bool check_global(double lat, double lon, double alt,
				double lat0, double lon0, double alt0)
{
	bool reached;
	if(
		((lat - 0.000001) < lat0)
	 && (lat0 < (lat + 0.000001)) 
	 && ((lon - 0.000001) < lon0)
	 && (lon0 < (lon + 0.000001))
	 && ((alt - 0.1) < alt0)
	 && (alt0 < (alt + 0.1))
	)
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}

void input_local_target()
{
	std::cout << "Input Local position(s)" << std::endl;
	std::cout << "Number of target(s): "; std::cin >> target_num;
	for (int i = 0; i < target_num; i++)
	{
		std::cout << "Target (" << i+1 << ") position (in meter):" <<std::endl; 
		std::cout << "pos_x_" << i+1 << ":"; std::cin >> target_pos[i][0];
		std::cout << "pos_y_" << i+1 << ":"; std::cin >> target_pos[i][1];
		std::cout << "pos_z_" << i+1 << ":"; std::cin >> target_pos[i][2];
		updates_local(i, target_pos[i][0], target_pos[i][1], target_pos[i][2]);
		// std::cout << "Target (" << i+1 << ") orientation (in degree):" <<std::endl; 
		target_pos[i][3] = 0;
		target_pos[i][4] = 0;
		target_pos[i][5] = 0;
		// std::cout << "yaw_" << i+1 << ":"; std::cin >> target_pos[i][5];
	}
}

void input_global_target()
{
	std::cout << "Input GPS position(s)" << std::endl;
	std::cout << "Number of goal(s): "; std::cin >> goal_num;
	for (int i = 0; i < goal_num; i++)
	{
		std::cout << "Goal ("<< i+1 <<") position:" << std::endl;
		std::cout << "Latitude  " << i+1 << " (in degree): "; std::cin >> goal_pos[i][0];
		std::cout << "Longitude " << i+1 << " (in degree): "; std::cin >> goal_pos[i][1];
		std::cout << "Altitude  " << i+1 << "  (in meter): "; std::cin >> goal_pos[i][2];
		updates_global(i, goal_pos[i][0], goal_pos[i][1], goal_pos[i][2]);
	}
}

void input_target()
{
	char c;
	std::cout << "Waypoint type: (1) Local || (2) Global ? (1/2) \n"; std::cin >> c;
	if (c == '1')
	{
		input_local_target();
		input_type = true;
	}
	else if (c == '2')
	{
		input_global_target();
		input_type = false;
	}
	else 
	{
		input_target();
	}
}

double degree(double rad)
{
	double radian_to_degree = (rad*180)/PI;
	return radian_to_degree;
}

double radian(double deg)
{
	double degree_to_radian = (deg*PI)/180;
	return degree_to_radian;
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

// void creates(std::string name, double x_log, double y_log, double z_log, 
//             double lat_log, double long_log, double alt_log) 
// { 
// 	std::fstream file; 

// 	file.open("ivsr_logs.csv", std::ios::out | std::ios::app); 

// 	file << ", , , , , , \n";
//     file << "Point, x, y, z, lat, long, alt \n";
//     file << name << ", " << std::fixed << std::setprecision(8) << x_log << ", " 
// 						 << std::fixed << std::setprecision(8) << y_log << ", " 
// 						 << std::fixed << std::setprecision(8) << z_log << ", "
// 						 << std::fixed << std::setprecision(8) << lat_log << ", " 
// 						 << std::fixed << std::setprecision(8) << long_log << ", " 
// 						 << std::fixed << std::setprecision(8) << alt_log << "\n";
	
//     file.close(); 
// } 

// void updates(std::string name, double x_log, double y_log, double z_log, 
//                    double lat_log, double long_log, double alt_log) 
// { 
// 	std::fstream file; 

//     file.open("ivsr_logs.csv", std::ios::out | std::ios::app); 

//     file << name << ", " << std::fixed << std::setprecision(8) << x_log << ", " 
// 						 << std::fixed << std::setprecision(8) << y_log << ", " 
// 						 << std::fixed << std::setprecision(8) << z_log << ", "
// 						 << std::fixed << std::setprecision(8) << lat_log << ", " 
// 						 << std::fixed << std::setprecision(8) << long_log << ", " 
// 						 << std::fixed << std::setprecision(8) << alt_log << "\n";

// 	file.close(); 
// } 

// void updates_check(int i, double x_log, double y_log, double z_log, 
//                    double lat_log, double long_log, double alt_log) 
// { 
// 	std::fstream file; 

//     file.open("ivsr_logs.csv", std::ios::out | std::ios::app); 

//     file << "checkpoint " << i << ", " << std::fixed << std::setprecision(8) << x_log << ", " 
// 									   << std::fixed << std::setprecision(8) << y_log << ", " 
// 									   << std::fixed << std::setprecision(8) << z_log << ", "
//          							   << std::fixed << std::setprecision(8) << lat_log << ", " 
// 									   << std::fixed << std::setprecision(8) << long_log << ", " 
// 									   << std::fixed << std::setprecision(8) << alt_log << "\n";

// 	file.close(); 
// } 

// void updates_local(int num, double x, double y, double z) 
// { 
// 	std::fstream file; 

//     file.open("ivsr_logs.csv", std::ios::out | std::ios::app); 

//     file << "local target " << num << ", " << std::fixed << std::setprecision(8) << x << ", " 
// 										   << std::fixed << std::setprecision(8) << y << ", " 
// 										   << std::fixed << std::setprecision(8) << z << ", , , \n";
// 	file.close();
// } 

// void updates_global(int num, double lat, double lon, double alt) 
// { 
// 	std::fstream file; 

//     file.open("ivsr_logs.csv", std::ios::out | std::ios::app); 

//     file << "global target " << num <<  ", , , , " << std::fixed << std::setprecision(8) << lat << ", " 
// 												   << std::fixed << std::setprecision(8) << lon << ", " 
// 												   << std::fixed << std::setprecision(8) << alt << "\n";

// 	file.close();
// }