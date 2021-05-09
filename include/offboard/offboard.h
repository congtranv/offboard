#ifndef OFFBOARD_H
#define OFFBOARD_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/Bool.h>
/******* local position *******/
#include <geometry_msgs/PoseStamped.h>
/******* global position *******/
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/GPSRAW.h>
/******* tranformation *******/
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
/******* C++ ******/
#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h> 

const double PI = 3.141592653589793238463;
const double eR = 6378.137;         // earth radius in km

const double a = 6378137.0;         // WGS-84 Earth semimajor axis (m)
const double b = 6356752.314245;    // Derived Earth semiminor axis (m)
const double f = (a - b) / a;       // Ellipsoid Flatness
const double f_inv = 1.0 / f;       // Inverse flattening

const double a_sq = a * a;
const double b_sq = b * b;
const double e_sq = f * (2 - f);    // Square of Eccentricity

bool global_position_received = false; // check receive global position
bool gps_position_received = false; // check receive GPS raw position

mavros_msgs::State current_state_; // check connection to pixhawk
mavros_msgs::GPSRAW gps_position_; // gps raw position from pixhawk
geometry_msgs::PoseStamped current_pose_; // current local position
sensor_msgs::NavSatFix global_position_; // global position

std_msgs::Bool human_trigger_; // trigger when detected human

// trajectory generation **********************************************************************
const int dimension = 3;
const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
const double v_max = 0.7;
const double a_max = 1.0;
const int N = 10;

mav_trajectory_generation::Vertex::Vector vertices;
mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
std::vector<double> segment_times;
mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
mav_trajectory_generation::Segment::Vector segments;
mav_trajectory_generation::Trajectory trajectory;

visualization_msgs::MarkerArray markers;
double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
std::string frame_id = "map";

void polynomial_optimization_linear(geometry_msgs::PoseStamped start_pose, geometry_msgs::PoseStamped middle_pose, geometry_msgs::PoseStamped end_pose);
//*********************************************************************************************

sensor_msgs::NavSatFix goalTransfer(double lat, double lon, double alt); // transfer lat, lon, alt setpoint to same message type with global_position
geometry_msgs::PoseStamped targetTransfer(double x, double y, double z); // transfer x, y, z setpoint to same message type with local_position

inline void state_cb(const mavros_msgs::State::ConstPtr& msg);
inline void localPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
inline void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg);
inline void gpsPosition_cb(const mavros_msgs::GPSRAW::ConstPtr& msg); 

inline void trigger_cb(const std_msgs::Bool::ConstPtr& msg);

class OffboardControl
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;
    ros::Subscriber local_pose_sub_;
    ros::Subscriber global_pos_sub_;
    ros::Subscriber gps_pos_sub_;

    ros::Subscriber trigger_sub_;

    ros::Publisher local_pos_pub_;

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;

    float t_hover_; // time set to drone when hovering
    double gps_lat, gps_lon, gps_alt; // gps latitude, longitude, altitude
    float local_error_, global_error_; // offset for checking when drone go to near setpoints for local and global 
    float check_error_; // offset for checking when drone go to near setpoints in main program
    double distance_; // calculate the distance from current position to setpoint
    double x_off_[100], y_off_[100], z_off_[100]; // store a series of offset between local position and ENU converted from global position
    double x_offset_, y_offset_, z_offset_;

    double vel_desired_; // desired velocity
    std::vector<double> vel_;

    bool final_check_ = false; // true == reached final point || false == NOT final point
    bool local_input_ = true; // true == input local || false == input global setpoints
    // bool human_trigger_ = false; 

    int target_num_; // number of local position setpoints
    std::vector<double> in_x_pos_; // vector to store in x position
    std::vector<double> in_y_pos_; // vector to store in y position
    std::vector<double> in_z_pos_; // vector to store in z position
    int goal_num_; // number of global position setpoints
    std::vector<double> in_latitude_; // vector to store latitude
    std::vector<double> in_longitude_; // vector to store longitude
    std::vector<double> in_altitude_; // vector to store altitude

    mavros_msgs::SetMode set_mode_; // set OFFBOARD mode in simulation
    geometry_msgs::PoseStamped target_pose_; // target local setpoint
    geographic_msgs::GeoPoseStamped goal_position_; // goal position 
    geometry_msgs::Point enu_g_, enu_c_, enu_r_; // local ENU points: converted from GPS goal and current
    geographic_msgs::GeoPoint wgs84_t_, wgs84_c_; // global WGS84 point: convert from ENU target and current
    sensor_msgs::NavSatFix ref_; // reference point to convert ECEF to ENU and vice versa
    geometry_msgs::Point offset_; // average offset in each axis

    bool check_position(float error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);
    bool check_orientation(float error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);
    void input_target();
    void input_local_target(); 
    void input_global_target();

    inline double degree(double rad); 
    inline double radian(double deg);

    double measureGPS(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2);
    double distanceLocal(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);
    
    geometry_msgs::Point WGS84ToECEF(sensor_msgs::NavSatFix wgs84);
    geographic_msgs::GeoPoint ECEFToWGS84(geometry_msgs::Point ecef);
    geometry_msgs::Point ECEFToENU(geometry_msgs::Point ecef, sensor_msgs::NavSatFix ref);
    geometry_msgs::Point ENUToECEF(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref);
    geometry_msgs::Point WGS84ToENU(sensor_msgs::NavSatFix wgs84, sensor_msgs::NavSatFix ref);
    geographic_msgs::GeoPoint ENUToWGS84(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref);

public:
  
    void takeOff(ros::Rate rate);
    void hover(geometry_msgs::PoseStamped target, ros::Rate rate);
    void landing(geometry_msgs::PoseStamped setpoint, ros::Rate rate);
    void position_control(ros::NodeHandle nh, ros::Rate rate);
    std::vector<double> vel_limit(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);

};

#endif