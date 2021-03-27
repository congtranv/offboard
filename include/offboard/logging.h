#include <iostream>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>

void creates(void);
void get_GPS(mavros_msgs::GPSRAW, double*, double*, double*); 
void updates(std::string, geometry_msgs::PoseStamped, 
             geometry_msgs::Point, sensor_msgs::NavSatFix,
             mavros_msgs::GPSRAW, double);
void updates_check(int, geometry_msgs::PoseStamped, 
             geometry_msgs::Point, sensor_msgs::NavSatFix,
             mavros_msgs::GPSRAW, double);
void updates_local(int, double, double, double); 
void updates_global(int, double, double, double);

void creates_sensor(void);
void updates_sensor(std::string, sensor_msgs::Imu, sensor_msgs::MagneticField, 
                    sensor_msgs::FluidPressure, sensor_msgs::FluidPressure);
void updates_check_ss(int, sensor_msgs::Imu, sensor_msgs::MagneticField, 
                    sensor_msgs::FluidPressure, sensor_msgs::FluidPressure);                                 