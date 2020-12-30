#include "offboard/offboard.h"

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "logs");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, localPose_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix> 
            ("mavros/global_position/global", 10, globalPosition_cb);
    
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected)
	{
        std::cout << "[ INFO] Waiting for FCU connection...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] FCU connected \n";

    // wait for GPS information
    while (ros::ok() && !global_position_received) 
    {
        std::cout << "[ INFO] Waiting for GPS signal...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] GPS position received \n";
    std::cout << "[ INFO] Checking status...\n";
    ros::Duration(1).sleep();

	// check current pose
	for(int i = 100; ros::ok() && i > 0; --i)
	{
        std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", 
                     current_pose.pose.position.x, 
                     current_pose.pose.position.y, 
                     current_pose.pose.position.z);
		
        std::printf("Current GPS position: [%f, %f, %.3f]\n", 
                     global_position.latitude, 
                     global_position.longitude, 
                     global_position.altitude);

        std::cout << "\n";
		ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Check status done \n";
    creates("initial", current_pose.pose.position.x,
                        current_pose.pose.position.y,
                        current_pose.pose.position.z,
                        refpoint.latitude,
                        refpoint.longitude,
                        refpoint.altitude);
    while (ros::ok())
    {
        std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", 
                     current_pose.pose.position.x, 
                     current_pose.pose.position.y, 
                     current_pose.pose.position.z);
		
        std::printf("Current GPS position: [%f, %f, %.3f]\n", 
                     global_position.latitude, 
                     global_position.longitude, 
                     global_position.altitude);
        updates("flight", current_pose.pose.position.x,
                          current_pose.pose.position.y,
                          current_pose.pose.position.z,
                          global_position.latitude,
                          global_position.longitude,
                          global_position.altitude);
        ros::Duration(5).sleep();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}