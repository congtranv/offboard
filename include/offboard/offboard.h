/******* ros *******/
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/FluidPressure.h>
/******* local position *******/
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
/******* global position *******/
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPoint.h>

#include <mavros_msgs/GPSRAW.h>
/******* tranformation *******/
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
/******* C++ ******/
#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>

/****** DEFINE CONSTANTS ******/

const double PI  =3.141592653589793238463;
const double eR  =6378.137;         // earth radius in km

const double a = 6378137.0;         // WGS-84 Earth semimajor axis (m)
const double b = 6356752.314245;    // Derived Earth semiminor axis (m)
const double f = (a - b) / a;       // Ellipsoid Flatness
const double f_inv = 1.0 / f;       // Inverse flattening

const double a_sq = a * a;
const double b_sq = b * b;
const double e_sq = f * (2 - f);    // Square of Eccentricity

/****** DECLARE VARIANTS ******/
mavros_msgs::State current_state; // check connection to pixhawk
mavros_msgs::SetMode set_mode; // set OFFBOARD mode in simulation
std_msgs::Float64 rel_alt; // altitude relative to home
sensor_msgs::Imu imu_data; // imu date from pixhawk
sensor_msgs::MagneticField mag_data; // magnetometer data
sensor_msgs::FluidPressure static_press, diff_press; // pressure data
sensor_msgs::BatteryState current_batt; // battery data
float batt_percent; // baterry capacity calculated from data

bool global_position_received = false; // check receive global position
sensor_msgs::NavSatFix global_position; // global position from pixhawk via mavros
geographic_msgs::GeoPoseStamped goal_position; // goal position setpoint to pixhawk

bool gps_position_received = false; // check receive GPS raw position
mavros_msgs::GPSRAW gps_position; // gps raw position from pixhawk
double gps_lat, gps_lon, gps_alt; // gps latitude, longitude, altitude to calculate

geometry_msgs::PoseStamped current_pose; // current local position
geometry_msgs::PoseStamped target_pose; // target local setpoint
geometry_msgs::PoseStamped takeoff_pose; // position to takeoff
geometry_msgs::PoseStamped traj_pose; // ref position from trajectory generation

geometry_msgs::PoseStamped zero_pose; 
geometry_msgs::Point zero_point; 

tf::Quaternion q; // quaternion for transforming to RPY

int target_num; // number of local position setpoints
std::vector<double> in_x_pos; // vector to store in x axis position
std::vector<double> in_y_pos; // vector to store in y axis position
std::vector<double> in_z_pos; // vector to store in z axis position
int goal_num; // number of global position setpoints
std::vector<double> in_latitude; // vector to store latitude
std::vector<double> in_longitude; // vector to store longitude
std::vector<double> in_altitude; // vector to store altitude

float local_error, global_error; // offset for checking when drone go to near setpoints for local and global 
float check_error; // offset for checking when drone go to near setpoints in main program

double distance; // calculate the distance from current position to setpoint

bool local_input = true; // true == input local || false == input global setpoints
bool final_check = false; // true == reached final point || false == NOT final point

ros::Time t_check; // store ros time when need a time trigger
float t_hover; // time set to drone when hovering

double v_x, v_y, v_z; // speed components in x, y and z axis to generate trajectory ref points

geometry_msgs::Point enu_goal, enu_curr, enu_ref; // local ENU points: converted from GPS goal and current
geographic_msgs::GeoPoint wgs84_target, wgs84_curr; // global WGS84 point: convert from ENU target and current
sensor_msgs::NavSatFix refpoint; // reference point to convert ECEF to ENU and vice versa
double x_off[100], y_off[100], z_off[100]; // store a series of offset between local position and ENU converted from global position
geometry_msgs::Point offset; // average offset in each axis

/****** DEFINE FUNCTIONS ******/

/* check_position: check when drone reached the target local positions   
   input: error, current_pose, target_pose - return: true or false */
bool check_position(float, geometry_msgs::PoseStamped,
				    geometry_msgs::PoseStamped);

/* check_orientation: check when drone reached the target orientations     
   input: error, current_pose, target_pose - return: true or false */
bool check_orientation(float, geometry_msgs::PoseStamped,
				   	   geometry_msgs::PoseStamped);

/* check_global: check when drone reached the GPS goal positions 
   input: (current) lat0, lon0, alt0, (target) lat, lon, alt - return: true or false */
bool check_global(double, double, double,
				  double, double, double);

/* input_local_target: input the local (x, y, z) point(s) */
void input_local_target(void);

/* input_global_target: input the GPS [latitude, longitude, altitude] point(s) */
void input_global_target(void);

/* input_target: choose the local or global input targets */
void input_target(void);

/* degree: convert angle from radian to degree 
   input: angle in radian - return: angle in degree */
double degree(double);

/* radian: convert angle from degree to radian 
   input: angle in degree - return: angle in radian */
double radian(double);

/* measureGPS: measure the distance between 2 GPS points that use haversine formula
   input: (GPS1) lat1, lon1, alt1, (GPS2) lat2, lon2, alt2 - return: distance in meters */
double measureGPS(double, double, double, double, double, double);

/* distanceLocal: measure the distance between 2 local points
   input: current pose, target pose - return: distance in meters */
double distanceLocal(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped);

/* velocityGenerate: from current position, target postion and v_desired to generate the axis velocities
   input: current pose, target pose - return: vx, vy, vz */
void velociyGenerate(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, 
					  double*, double*, double*);

// /* get_GPS: get GPS lat, lon, alt from GPS RAW topic */
// void get_GPS(int, int, int, double*, double*, double*); 

/* WGS84ToECEF: Converts the WGS-84 Geodetic point (latitude, longitude, altitude) 
   to Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) */
geometry_msgs::Point WGS84ToECEF(double, double, double);

/* ECEFToWGS84: Converts the Earth-Centered Earth-Fixed coordinates (x, y, z) to  
   WGS-84 Geodetic point (latitude, longitude, altitude) */
geographic_msgs::GeoPoint ECEFToWGS84(double, double, double);

/* ECEFToENU: Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) 
   to East-North-Up coordinates in a Local Tangent Plane that is centered at the   
   (WGS-84) Geodetic point (lat0, lon0, alt0) */
geometry_msgs::Point ECEFToENU(double, double, double, double, double, double);

/* ENUToECEF: Converts East-North-Up coordinates (xEast, yNorth, zUp) in a Local   
   Tangent Plane that is centered at  (WGS-84) Geodetic point (lat0, lon0, alt0)   
   to the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) */
geometry_msgs::Point ENUToECEF(double, double, double, double, double, double);

/* WGS84ToENU: Converts the geodetic WGS-84 coordinated (lat, lon, alt) to         
   East-North-Up coordinates in a Local Tangent Plane that is centered at the      
   (WGS-84) Geodetic point (lat0, lon0, alt0) */
geometry_msgs::Point WGS84ToENU(double, double, double, double, double, double);

/* ENUToWGS84: Converts the East-North-Up coordinates in a Local Tangent Plane to  
   geodetic WGS-84 coordinated (lat, lon, alt) that is centered at the (WGS-84)    
   Geodetic point (lat0, lon0, alt0) */
geographic_msgs::GeoPoint ENUToWGS84(double, double, double, double, double, double);

/****** CALLBACK FUNCTIONS ******/

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}
void relativeAlt_cb(const std_msgs::Float64::ConstPtr& msg)
{
    rel_alt = *msg;
}
void imuData_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
	imu_data = *msg;
}
void magData_cb(const sensor_msgs::MagneticField::ConstPtr& msg)
{
	mag_data = *msg;
}
void staticPress_cb(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
	static_press = *msg;
}
void diffPress_cb(const sensor_msgs::FluidPressure::ConstPtr& msg)
{
	diff_press = *msg;
}
void localPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
    global_position = *msg;
    global_position_received = true;
}
void gpsPosition_cb(const mavros_msgs::GPSRAW::ConstPtr& msg) 
{
    gps_position = *msg;
	gps_position_received = true;
}
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg) 
{
    current_batt = *msg;
}