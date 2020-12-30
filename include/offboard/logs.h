#include <iostream>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>

void creates(std::string, double, double, double, 
            double, double, double);
void updates(std::string, double, double, double, 
                   double, double, double);
void updates_check(int, double, double, double, 
                   double, double, double);
void updates_local(int, double, double, double); 
void updates_global(int, double, double, double);

void creates(std::string name, double x_log, double y_log, double z_log, 
            double lat_log, double long_log, double alt_log) 
{ 
	std::fstream file; 

	file.open("ivsr_logs.csv", std::ios::out | std::ios::app); 

	file << ", , , , , , \n";
    file << "Point, x, y, z, lat, long, alt \n";
    file << name << ", " << std::fixed << std::setprecision(6) << x_log << ", " 
						 << std::fixed << std::setprecision(6) << y_log << ", " 
						 << std::fixed << std::setprecision(6) << z_log << ", "
						 << std::fixed << std::setprecision(6) << lat_log << ", " 
						 << std::fixed << std::setprecision(6) << long_log << ", " 
						 << std::fixed << std::setprecision(3) << alt_log << "\n";
	
    file.close(); 
} 

void updates(std::string name, double x_log, double y_log, double z_log, 
                   double lat_log, double long_log, double alt_log) 
{ 
	std::fstream file; 

    file.open("ivsr_logs.csv", std::ios::out | std::ios::app); 

    file << name << ", " << std::fixed << std::setprecision(6) << x_log << ", " 
						 << std::fixed << std::setprecision(6) << y_log << ", " 
						 << std::fixed << std::setprecision(6) << z_log << ", "
						 << std::fixed << std::setprecision(6) << lat_log << ", " 
						 << std::fixed << std::setprecision(6) << long_log << ", " 
						 << std::fixed << std::setprecision(3) << alt_log << "\n";

	file.close(); 
} 

void updates_check(int i, double x_log, double y_log, double z_log, 
                   double lat_log, double long_log, double alt_log) 
{ 
	std::fstream file; 

    file.open("ivsr_logs.csv", std::ios::out | std::ios::app); 

    file << "checkpoint " << i << ", " << std::fixed << std::setprecision(6) << x_log << ", " 
									   << std::fixed << std::setprecision(6) << y_log << ", " 
									   << std::fixed << std::setprecision(6) << z_log << ", "
         							   << std::fixed << std::setprecision(6) << lat_log << ", " 
									   << std::fixed << std::setprecision(6) << long_log << ", " 
									   << std::fixed << std::setprecision(3) << alt_log << "\n";

	file.close(); 
} 

void updates_local(int num, double x, double y, double z) 
{ 
	std::fstream file; 

    file.open("ivsr_logs.csv", std::ios::out | std::ios::app); 

    file << "local target " << num << ", " << std::fixed << std::setprecision(6) << x << ", " 
										   << std::fixed << std::setprecision(6) << y << ", " 
										   << std::fixed << std::setprecision(6) << z << ", , , \n";
	file.close();
} 

void updates_global(int num, double lat, double lon, double alt) 
{ 
	std::fstream file; 

    file.open("ivsr_logs.csv", std::ios::out | std::ios::app); 

    file << "global target " << num <<  ", , , , " << std::fixed << std::setprecision(6) << lat << ", " 
												   << std::fixed << std::setprecision(6) << lon << ", " 
												   << std::fixed << std::setprecision(6) << alt << "\n";

	file.close();
}