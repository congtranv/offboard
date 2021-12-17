#ifndef OFFBOARD_H_
#define OFFBOARD_H_

#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_datatypes.h>

#include<mavros_msgs/State.h>
#include<mavros_msgs/SetMode.h>
#include<mavros_msgs/CommandBool.h>
#include<std_msgs/Bool.h>
#include<std_msgs/Float64.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/TwistStamped.h>
#include<geographic_msgs/GeoPoseStamped.h>
#include<sensor_msgs/NavSatFix.h>

#include<iostream>
#include<cmath>
#include<cstdio>
#include<vector>

class OffboardControl
{
  public:
	OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
	~OffboardControl();
  private:
	/* CONSTANT */
	const double PI = 3.141592653589793238463; // PI
	const double eR = 6378.137;         // earth radius in km
	const double a = 6378137.0;         // WGS-84 Earth semimajor axis (m)
	const double b = 6356752.314245;    // Derived Earth semiminor axis (m)
	const double f = (a - b) / a;       // Ellipsoid Flatness
	const double f_inv = 1.0 / f;       // Inverse flattening
	const double a_sq = a * a;
	const double b_sq = b * b;
	const double e_sq = f * (2 - f);    // Square of Eccentricity

	ros::NodeHandle nh_;
	ros::NodeHandle nh_private_;

	ros::Subscriber state_sub_; // current state subscriber
	ros::Subscriber enu_pose_sub_; // current enu pose subscriber
	ros::Subscriber gps_position_sub_; // current gps position subscriber
	ros::Subscriber velocity_body_sub_; 
	ros::Subscriber opt_point_sub_; // optimization point from planner subscriber

	ros::Publisher setpoint_pose_pub_; // publish target pose to drone
	ros::ServiceClient set_mode_client_; // set OFFBOARD mode in simulation
	ros::ServiceClient arming_client_; // call arm command in simulation

	mavros_msgs::State current_state_;
	geometry_msgs::PoseStamped current_enu_pose_;
	geometry_msgs::PoseStamped home_enu_pose_;
	geometry_msgs::PoseStamped target_enu_pose_;
	geometry_msgs::TwistStamped current_body_vel_;
	geometry_msgs::Point opt_point_;
	sensor_msgs::NavSatFix current_gps_position_;
	sensor_msgs::NavSatFix home_gps_position_;
	geographic_msgs::GeoPoseStamped goal_gps_position_;
	mavros_msgs::SetMode flight_mode_;
	sensor_msgs::NavSatFix ref_gps_position_;
	geometry_msgs::Point enu_position_offset_;

	bool opt_point_received_ = false;
	bool gps_received_ = false; 
	bool final_position_reached_ = false; // true = reach final setpoint || false = other setpoints
	bool delivery_mode_enable_;
	bool simulation_mode_enable_;
	bool return_home_mode_enable_;
	
	int num_of_enu_target_;
	std::vector<double> x_target_;
	std::vector<double> y_target_; 
	std::vector<double> z_target_;
	int num_of_gps_goal_;
	std::vector<double> lat_goal_; 
	std::vector<double> lon_goal_;
	std::vector<double> alt_goal_;
	
	double geometric_error_, target_error_, goal_error_, land_error_;
	double distance_;
	double x_off_[100], y_off_[100], z_off_[100];
	double x_offset_, y_offset_, z_offset_;
	double z_takeoff_;

	double vel_desired_, land_vel_, return_vel_;
	geometry_msgs::Vector3 components_vel_;
	double hovering_time_, hover_time_, takeoff_hover_time_, unpack_time_;
	ros::Time operation_time_1_, operation_time_2_;

	void waitForPredicate(double hz);
	void setOffboardStream(double hz, geometry_msgs::PoseStamped first_target);
	void waitForArmAndOffboard(double hz);
	void waitForStable(double hz);
	void stateCallback(const mavros_msgs::State::ConstPtr& msg);
	void enuPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
	void velocityBodyCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void optPointCallback(const geometry_msgs::Point::ConstPtr& msg);
	
	// inline double avgBodyVelocity(geometry_msgs::TwistStamped vel)
	// {
	// 	return sqrt((vel.twist.linear.x*vel.twist.linear.x)+(vel.twist.linear.y*vel.twist.linear.y)+(vel.twist.linear.z*vel.twist.linear.z));
	// }

	inline double degreeOf(double rad)
	{
		return (rad*180)/PI;
	}

	inline double radianOf(double deg)
	{
		return (deg*PI)/180;
	}
	
	template <class T>
    inline T sqr(T x)
	{
        return x*x;
    }; 

	sensor_msgs::NavSatFix goalTransfer(double lat, double lon, double alt); // transfer lat, lon, alt setpoint to same message type with global_position
	geometry_msgs::PoseStamped targetTransfer(double x, double y, double z); // transfer x, y, z setpoint to same message type with local_position
	// geometry_msgs::Point pointFromPoseSubtraction(geometry_msgs::PoseStamped pose1, geometry_msgs::PoseStamped pose2);
	// geometry_msgs::Point pointFromDouble(double x, double y, double z);
	// void doubleFromPoint(double x, double y, double z, geometry_msgs::Point point);

	bool checkPositionError(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);
	bool checkOrientationError(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);
	
	double distanceBetween(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);
	geometry_msgs::Vector3 velComponentsCalc(double v_desired, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);

	void takeOff(geometry_msgs::PoseStamped setpoint, double hover_time);
	void hovering(geometry_msgs::PoseStamped setpoint, double hover_time);
	void landing(geometry_msgs::PoseStamped setpoint);
	void returnHome(geometry_msgs::PoseStamped home_pose);
	void delivery(geometry_msgs::PoseStamped unpack_pose, double unpack_time);
	
	void inputSetpoint();
	void inputENU();
	void inputGPS();
	void inputPlanner();

	void enuFlight();
	void gpsFlight();
	void plannerFlight();

	geometry_msgs::Point WGS84ToECEF(sensor_msgs::NavSatFix wgs84);
	geographic_msgs::GeoPoint ECEFToWGS84(geometry_msgs::Point ecef);
	geometry_msgs::Point ECEFToENU(geometry_msgs::Point ecef, sensor_msgs::NavSatFix ref);
	geometry_msgs::Point ENUToECEF(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref);
	geometry_msgs::Point WGS84ToENU(sensor_msgs::NavSatFix wgs84, sensor_msgs::NavSatFix ref);
	geographic_msgs::GeoPoint ENUToWGS84(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref);
};


#endif