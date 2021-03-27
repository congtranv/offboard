#include <offboard/offboard.h>
#include <offboard/logging.h>

void creates()
{ 
	std::fstream file; 
	file.open("position.csv", std::ios::out | std::ios::app); 
    
	file << "Point, x (m), y (m), z (m), x_convert (m), y_convert (m), z_convert (m), lat (degree), lon (degree), alt (m), gps_lat, gps_lon, gps_alt, rel alt (m) \n";
    file << ", , , , , , , , , , , , , \n";

    file.close(); 
} 

void get_GPS(mavros_msgs::GPSRAW gps, double* lat, double* lon, double* alt)
{
    int gps_lat = gps.lat;
    int gps_lon = gps.lon;
    int gps_alt = gps.alt;
	*lat = double(gps_lat)/10000000;
    *lon = double(gps_lon)/10000000;
    *alt = double(gps_alt)/1000;
}

void updates(std::string name, geometry_msgs::PoseStamped current, 
             geometry_msgs::Point enu, sensor_msgs::NavSatFix global,
             mavros_msgs::GPSRAW gps, double rel_alt) 
{ 
    double x_log = current.pose.position.x;
    double y_log = current.pose.position.y;
    double z_log = current.pose.position.z;
    double x_convert = enu.x;
    double y_convert = enu.y;
    double z_convert = enu.z;
    double lat_log = global.latitude;
    double lon_log = global.longitude;
    double alt_log = global.altitude;
    double lat_gps, lon_gps, alt_gps;
    get_GPS(gps, &lat_gps, &lon_gps, &alt_gps);

	std::fstream file; 
    file.open("position.csv", std::ios::out | std::ios::app); 

    file << name << ", " << std::fixed << std::setprecision(8) << x_log << ", " 
						 << std::fixed << std::setprecision(8) << y_log << ", " 
						 << std::fixed << std::setprecision(8) << z_log << ", "
                         << std::fixed << std::setprecision(8) << x_convert << ", " 
						 << std::fixed << std::setprecision(8) << y_convert << ", " 
						 << std::fixed << std::setprecision(8) << z_convert << ", "
						 << std::fixed << std::setprecision(8) << lat_log << ", " 
						 << std::fixed << std::setprecision(8) << lon_log << ", " 
						 << std::fixed << std::setprecision(8) << alt_log << ", "
                         << std::fixed << std::setprecision(8) << lat_gps << ", "
                         << std::fixed << std::setprecision(8) << lon_gps << ", "
                         << std::fixed << std::setprecision(8) << alt_gps << ", "
                         << std::fixed << std::setprecision(8) << rel_alt << "\n";

	file.close(); 
} 

void updates_check(int i, geometry_msgs::PoseStamped current, 
             geometry_msgs::Point enu, sensor_msgs::NavSatFix global,
             mavros_msgs::GPSRAW gps, double rel_alt) 
{ 
    double x_log = current.pose.position.x;
    double y_log = current.pose.position.y;
    double z_log = current.pose.position.z;
    double x_convert = enu.x;
    double y_convert = enu.y;
    double z_convert = enu.z;
    double lat_log = global.latitude;
    double lon_log = global.longitude;
    double alt_log = global.altitude;
    double lat_gps, lon_gps, alt_gps;
    get_GPS(gps, &lat_gps, &lon_gps, &alt_gps);

	std::fstream file; 
    file.open("position.csv", std::ios::out | std::ios::app); 

    file << "checkpoint " << i << ", " << std::fixed << std::setprecision(8) << x_log << ", " 
									   << std::fixed << std::setprecision(8) << y_log << ", " 
									   << std::fixed << std::setprecision(8) << z_log << ", "
                                       << std::fixed << std::setprecision(8) << x_convert << ", " 
                              	       << std::fixed << std::setprecision(8) << y_convert << ", " 
						               << std::fixed << std::setprecision(8) << z_convert << ", "
         							   << std::fixed << std::setprecision(8) << lat_log << ", " 
									   << std::fixed << std::setprecision(8) << lon_log << ", " 
									   << std::fixed << std::setprecision(8) << alt_log << ", "
                                       << std::fixed << std::setprecision(8) << lat_gps << ", "
                                       << std::fixed << std::setprecision(8) << lon_gps << ", "
                                       << std::fixed << std::setprecision(8) << alt_gps << ", "
                                       << std::fixed << std::setprecision(8) << rel_alt << "\n";
	file.close(); 
} 

void updates_local(int num, double x, double y, double z) 
{ 
	std::fstream file; 
    file.open("position.csv", std::ios::out | std::ios::app); 
    
    file << "local target " << num << ", " << std::fixed << std::setprecision(8) << x << ", " 
										   << std::fixed << std::setprecision(8) << y << ", " 
										   << std::fixed << std::setprecision(8) << z << ", , , \n";
	file.close();
} 

void updates_global(int num, double lat, double lon, double alt) 
{ 
	std::fstream file; 
    file.open("position.csv", std::ios::out | std::ios::app); 

    file << "global target " << num <<  ", , , , , , , " << std::fixed << std::setprecision(8) << lat << ", " 
												         << std::fixed << std::setprecision(8) << lon << ", " 
												         << std::fixed << std::setprecision(8) << alt << "\n";

	file.close();
}

void creates_sensor()
{
	std::fstream file; 
	file.open("sensor.csv", std::ios::out | std::ios::app); 

	file << "Point, angular vel x (rad/s), angular vel y (rad/s), angular vel z (rad/s), " 
         << "linear acc x (m/s^2), linear acc y (m/s^2), linear acc z (m/s^2), "
         << "mag field x (T), mag field y (T), mag field z (T), static press (Pa), diff press (Pa) \n";
    file << ", , , , , , , , , , , \n";

    file.close(); 
}

void updates_sensor(std::string name, sensor_msgs::Imu imu, sensor_msgs::MagneticField mag, 
                    sensor_msgs::FluidPressure spress, sensor_msgs::FluidPressure dpress)
{
    double av_x = imu.angular_velocity.x;
    double av_y = imu.angular_velocity.y;
    double av_z = imu.angular_velocity.z; 
    double la_x = imu.linear_acceleration.x;
    double la_y = imu.linear_acceleration.y;
    double la_z = imu.linear_acceleration.z;
    double magx = mag.magnetic_field.x;
    double magy = mag.magnetic_field.y;
    double magz = mag.magnetic_field.z;
    double static_press = spress.fluid_pressure;
    double diff_press = dpress.fluid_pressure;

    std::fstream file; 
	file.open("sensor.csv", std::ios::out | std::ios::app); 

    file << name << ", " << std::fixed << std::setprecision(8) << av_x << ", " 
						 << std::fixed << std::setprecision(8) << av_y << ", " 
						 << std::fixed << std::setprecision(8) << av_z << ", "
						 << std::fixed << std::setprecision(8) << la_x << ", " 
						 << std::fixed << std::setprecision(8) << la_y << ", " 
						 << std::fixed << std::setprecision(8) << la_z << ", "
                         << std::fixed << std::setprecision(8) << magx << ", "
                         << std::fixed << std::setprecision(8) << magy << ", "
                         << std::fixed << std::setprecision(8) << magz << ", "
                         << std::fixed << std::setprecision(8) << static_press << ", "
                         << std::fixed << std::setprecision(8) << diff_press << "\n";
    file.close(); 
}
void updates_check_ss(int i, sensor_msgs::Imu imu, sensor_msgs::MagneticField mag, 
                    sensor_msgs::FluidPressure spress, sensor_msgs::FluidPressure dpress)
{
    double av_x = imu.angular_velocity.x;
    double av_y = imu.angular_velocity.y;
    double av_z = imu.angular_velocity.z; 
    double la_x = imu.linear_acceleration.x;
    double la_y = imu.linear_acceleration.y;
    double la_z = imu.linear_acceleration.z;
    double magx = mag.magnetic_field.x;
    double magy = mag.magnetic_field.y;
    double magz = mag.magnetic_field.z;
    double static_press = spress.fluid_pressure;
    double diff_press = dpress.fluid_pressure;

    std::fstream file; 
	file.open("sensor.csv", std::ios::out | std::ios::app); 

    file << "checkpoint" << i << ", " 
                         << std::fixed << std::setprecision(8) << av_x << ", " 
						 << std::fixed << std::setprecision(8) << av_y << ", " 
						 << std::fixed << std::setprecision(8) << av_z << ", "
						 << std::fixed << std::setprecision(8) << la_x << ", " 
						 << std::fixed << std::setprecision(8) << la_y << ", " 
						 << std::fixed << std::setprecision(8) << la_z << ", "
                         << std::fixed << std::setprecision(8) << magx << ", "
                         << std::fixed << std::setprecision(8) << magy << ", "
                         << std::fixed << std::setprecision(8) << magz << ", "
                         << std::fixed << std::setprecision(8) << static_press << ", "
                         << std::fixed << std::setprecision(8) << diff_press << "\n";
	
    file.close(); 
}    