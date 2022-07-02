#include"offboard/offboard.h"

// OffboardControl::OffboardControl(){
// }

OffboardControl::OffboardControl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, bool input_setpoint):
  nh_(nh),
  nh_private_(nh_private),
  simulation_mode_enable_(false),
  delivery_mode_enable_(false),
  return_home_mode_enable_(false)
{
    state_sub_ = nh_.subscribe("/mavros/state", 10, &OffboardControl::stateCallback, this);
    odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 10, &OffboardControl::odomCallback, this);
    gps_position_sub_ = nh_.subscribe("/mavros/global_position/global", 10, &OffboardControl::gpsPositionCallback, this);
    opt_point_sub_ = nh_.subscribe("optimization_point", 10, &OffboardControl::optPointCallback, this);

    setpoint_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    nh_private_.param<bool>("/offboard_node/simulation_mode_enable", simulation_mode_enable_, simulation_mode_enable_);
    nh_private_.param<bool>("/offboard_node/delivery_mode_enable", delivery_mode_enable_, delivery_mode_enable_);
    nh_private_.param<bool>("/offboard_node/return_home_mode_enable", return_home_mode_enable_, return_home_mode_enable_);
    nh_private_.getParam("/offboard_node/number_of_target", num_of_enu_target_);
    nh_private_.getParam("/offboard_node/target_error", target_error_);
    nh_private_.getParam("/offboard_node/target_x_pos", x_target_);
    nh_private_.getParam("/offboard_node/target_y_pos", y_target_);
    nh_private_.getParam("/offboard_node/target_z_pos", z_target_);
    nh_private_.getParam("/offboard_node/target_yaw", yaw_target_);
    nh_private_.getParam("/offboard_node/number_of_goal", num_of_gps_goal_);
    nh_private_.getParam("/offboard_node/goal_error", goal_error_);
    nh_private_.getParam("/offboard_node/latitude", lat_goal_);
    nh_private_.getParam("/offboard_node/longitude", lon_goal_);
    nh_private_.getParam("/offboard_node/altitude", alt_goal_);
    nh_private_.getParam("/offboard_node/z_takeoff", z_takeoff_);
    nh_private_.getParam("/offboard_node/z_delivery", z_delivery_);
    nh_private_.getParam("/offboard_node/land_error", land_error_);
    nh_private_.getParam("/offboard_node/takeoff_hover_time", takeoff_hover_time_);
    nh_private_.getParam("/offboard_node/hover_time", hover_time_);
    nh_private_.getParam("/offboard_node/unpack_time", unpack_time_);
    nh_private_.getParam("/offboard_node/desired_velocity", vel_desired_);
    nh_private_.getParam("/offboard_node/land_velocity", land_vel_);
    nh_private_.getParam("/offboard_node/return_velcity", return_vel_);
    
    waitForPredicate(10.0);
    if(input_setpoint)
    {
        inputSetpoint();
    }
}

OffboardControl::~OffboardControl(){

}

/* wait for connect, GPS received, ...
   input: ros rate in hertz, at least 2Hz */
void OffboardControl::waitForPredicate(double hz)
{
    ros::Rate rate(hz);

    std::printf("\n[ INFO] Waiting for FCU connection \n");
    while(ros::ok() && !current_state_.connected)
	{
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("[ INFO] FCU connected \n");
    
    std::printf("[ INFO] Waiting for GPS signal \n");
    while (ros::ok() && !gps_received_) 
    {
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("[ INFO] GPS position received \n");
    if(simulation_mode_enable_)
    {
        std::printf("\n[ NOTICE] Prameter 'simulation_mode_enable' is set true\n");
        std::printf("          OFFBOARD node will automatic ARM and set OFFBOARD mode\n");
        std::printf("          Continue if run a simulation OR SHUTDOWN node if run in drone\n");
        std::printf("          Set parameter 'simulation_mode_enable' to false or not set (default = false)\n");
        std::printf("          and relaunch node for running in drone\n");
        std::printf("          > roslaunch offboard offboard.launch simulation_mode_enable:=false\n");
    }
    else
    {
        std::printf("\n[ NOTICE] Prameter 'simulation_mode_enable' is set false or not set (default = false)\n");
        std::printf("          OFFBOARD node will wait for ARM and set OFFBOARD mode from RC controller\n");
        std::printf("          Continue if run in drone OR shutdown node if run a simulation\n");
        std::printf("          Set parameter 'simulation_mode_enable' to true and relaunch node for simulation\n");
        std::printf("          > roslaunch offboard offboard.launch simulation_mode_enable:=true\n");
    }    
    operation_time_1_ = ros::Time::now();
}

/* send a few setpoints before publish
   input: ros rate in hertz (at least 2Hz) and first setpoint */
void OffboardControl::setOffboardStream(double hz, geometry_msgs::PoseStamped first_target)
{
    ros::Rate rate(hz);
    std::printf("[ INFO] Setting OFFBOARD stream \n");
    for(int i=50; ros::ok() && i>0; --i)
    {
        target_enu_pose_ = first_target;
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);
        ros::spinOnce();
        rate.sleep();
    }
    std::printf("\n[ INFO] OFFBOARD stream is set\n");
}

/* wait for ARM and OFFBOARD mode switch (in SITL case or HITL/Practical case)
   input: ros rate in hertz, at least 2Hz */
void OffboardControl::waitForArmAndOffboard(double hz)
{
    ros::Rate rate(hz);
    if(simulation_mode_enable_)
    {
        std::printf("\n[ INFO] Ready to takeoff\n");   
        while(ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD"))
        {    
            mavros_msgs::CommandBool arm_amd;
            arm_amd.request.value = true;
            if(arming_client_.call(arm_amd) && arm_amd.response.success)
            {
                ROS_INFO_ONCE("Vehicle armed");
            }
            else
            {
                ROS_INFO_ONCE("Arming failed");
            }

            mavros_msgs::SetMode offboard_setmode;
            offboard_setmode.request.base_mode = 0;
            offboard_setmode.request.custom_mode = "OFFBOARD";
            if(set_mode_client_.call(offboard_setmode) && offboard_setmode.response.mode_sent)
            {
                ROS_INFO_ONCE("OFFBOARD enabled");
            }
            else 
            {
                ROS_INFO_ONCE("Failed to set OFFBOARD");
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
    else
    {
        std::printf("\n[ INFO] Waiting switching (ARM and OFFBOARD mode) from RC\n");
        while(ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD"))
        { 
            ros::spinOnce();
            rate.sleep();
        }
    }
    
}

/* wait drone get a stable state
   input: ros rate in hertz, at least 2Hz */
void OffboardControl::waitForStable(double hz)
{
    ros::Rate rate(hz);
    std::printf("\n[ INFO] Waiting for stable state\n");

    ref_gps_position_ = current_gps_position_;
    geometry_msgs::Point converted_enu;
    for(int i=0; i<100; i++)
    {
        converted_enu = WGS84ToENU(current_gps_position_, ref_gps_position_);
        x_off_[i] = current_odom_.pose.pose.position.x - converted_enu.x;
        y_off_[i] = current_odom_.pose.pose.position.y - converted_enu.y;
        z_off_[i] = current_odom_.pose.pose.position.z - converted_enu.z;
        ros::spinOnce();
        rate.sleep();
    }
    for(int i=0; i<100; i++)
    {
        x_offset_ = x_offset_ + x_off_[i]/100;
        y_offset_ = y_offset_ + y_off_[i]/100;
        z_offset_ = z_offset_ + z_off_[i]/100;
    }
    std::printf("[ INFO] Got stable state\n");
    
    home_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z, yaw_);
    home_gps_position_ = current_gps_position_;
    std::printf("\n[ INFO] Got HOME position: [%.1f, %.1f, %.1f, %.1f]\n", home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, home_enu_pose_.pose.position.z, tf::getYaw(home_enu_pose_.pose.orientation));
    std::printf("        latitude : %.8f\n", home_gps_position_.latitude);
    std::printf("        longitude: %.8f\n", home_gps_position_.longitude);
    std::printf("        altitude : %.8f\n", home_gps_position_.altitude);
}

void OffboardControl::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void OffboardControl::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_odom_ = *msg;
    odom_received_ = true;
    tf::poseMsgToEigen(current_odom_.pose.pose, current_pose_);
    tf::vectorMsgToEigen(current_odom_.twist.twist.linear, current_velocity_);
    yaw_ = tf::getYaw(current_odom_.pose.pose.orientation);
    // std::cout << "\n[Debug] yaw from odom: " << degreeOf(yaw_) << "\n";
}
	
void OffboardControl::gpsPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_gps_position_ = *msg;
    gps_received_ = true;
}

void OffboardControl::optPointCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    opt_point_ = *msg;
    optimization_point_.push_back(*msg);
    opt_point_received_ = true;
}

/* manage input: select mode, setpoint type, ... */
void OffboardControl::inputSetpoint()
{
    std::printf("\n[ INFO] Please choose mode\n");
    char mode;
    std::printf("- Choose (1): Hovering\n");
    std::printf("- Choose (2): Mission\n");
    std::printf("(1/2): ");
    std::cin >> mode;

    if(mode == '1') // hovering
    {
        double x, y, z;
        double hover_time;
        std::printf("\n[ INFO] Mode 1: Hovering\n");
        std::printf(" Please enter the altitude you want to hover (in meters): ");
        std::cin >> z;
        std::printf(" Please enter the time you want to hover (in seconds): ");
        std::cin >> hover_time;
        x = current_odom_.pose.pose.position.x;
        y = current_odom_.pose.pose.position.y;
        
        setOffboardStream(10.0, targetTransfer(x, y, z));
        waitForArmAndOffboard(10.0);

        takeOff(targetTransfer(x, y, z), hover_time);
        landing(targetTransfer(x, y, 0.0));
    }
    else if(mode == '2')
    {
        std::printf("\n[ INFO] Mode 2: Mission\n");
        std::printf("[ INFO] Parameter 'return_home_mode_enable' is set %s\n", return_home_mode_enable_ ? "true":"false");
        std::printf("[ INFO] Parameter 'delivery_mode_enable' is set %s\n", delivery_mode_enable_ ? "true":"false");
        std::printf("\n[ INFO] Please choose mission type:\n");
        char c;
        std::printf("- Choose (1): Mission with ENU setpoint\n");
        std::printf("- Choose (2): Mission with GPS setpoint\n");
        std::printf("- Choose (3): Mission with Planner\n");
        std::printf("(1/2/3): ");
        std::cin >> c;
        if(c == '1')
        {
            std::printf("\n[ INFO] Current ENU position: [%.1f, %.1f, %.1f]\n", current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z);
            std::printf("[ INFO] Current Orientation RPY: [%.1f, %.1f, %.1f]\n", getRPY(current_odom_.pose.pose.orientation).x(), getRPY(current_odom_.pose.pose.orientation).y(), getRPY(current_odom_.pose.pose.orientation).z());
            inputENU(); 
        }
        else if(c == '2')
        {
            std::printf("\n[ INFO] Current GPS position: [%.8f, %.8f, %.3f]\n", current_gps_position_.latitude, current_gps_position_.longitude, current_gps_position_.altitude);
            inputGPS();
        }
        else if (c == '3')
        {
            inputPlanner();
        }
        else
        {
            std::printf("\n[ WARN] Not avaible mode\n");
            inputSetpoint();
        }
    }
    else
    {
        std::printf("\n[ WARN] Not avaible mode\n");
        inputSetpoint();
    }
}

/* manage input for ENU setpoint flight mode: manual input from keyboard, load setpoints */
void OffboardControl::inputENU()
{
    ros::Rate rate(10.0);
    char c;
    std::printf("\n[ INFO] Please choose input method:\n");
    std::printf("- Choose 1: Manual enter from keyboard\n");
    std::printf("- Choose 2: Load prepared from launch file\n");
    std::printf("(1/2): ");
    std::cin >> c;
    if(c == '1')
    {
        double x, y, z, yaw;
        std::printf("[ INFO] Manual enter ENU target position(s)\n");
        std::printf(" Number of target(s): "); 
        std::cin >> num_of_enu_target_;
        if(!x_target_.empty() || !y_target_.empty() || !z_target_.empty())
        {
            x_target_.clear();
            y_target_.clear();
            z_target_.clear();
        }
        for(int i=0; i<num_of_enu_target_; i++)
        {
            std::printf(" Target (%d) postion x, y, z (in meter): ", i+1);
            std::cin >> x >> y >> z;
            x_target_.push_back(x); 
            y_target_.push_back(y); 
            z_target_.push_back(z);
            ros::spinOnce();
            rate.sleep();
        }
        std::printf(" Error to check target reached (in meter): ");
        std::cin >> target_error_;
    }
    else if(c == '2')
    {
        std::printf("[ INFO] Loaded prepared setpoints [x, y, z, yaw]\n");
        for(int i=0; i<num_of_enu_target_; i++)
        {
            std::printf(" Target (%d): [%.1f, %.1f, %.1f]\n", i+1, x_target_[i], y_target_[i], z_target_[i]);

            ros::spinOnce();
            rate.sleep();
        }
        std::printf(" Error to check target reached: %.1f (m)\n", target_error_);
    }
    else
    {
        inputENU();
    }
    waitForStable(10.0);
    setOffboardStream(10.0,targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_));
    waitForArmAndOffboard(10.0);
    takeOff(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_), takeoff_hover_time_);
    std::printf("\n[ INFO] Flight with ENU setpoint\n");
    enuFlight();
}

/* manage input for ENU setpoint flight mode: manual input from keyboard, load setpoints include yaw angle */
/*
void OffboardControl::inputENU()
{
    ros::Rate rate(10.0);
    char c;
    std::printf("\n[ INFO] Please choose input method:\n");
    std::printf("- Choose 1: Manual enter from keyboard\n");
    std::printf("- Choose 2: Load prepared from launch file\n");
    std::printf("(1/2): ");
    std::cin >> c;
    if(c == '1')
    {
        double x, y, z, yaw;
        std::printf("[ INFO] Manual enter ENU target position(s)\n");
        std::printf(" Number of target(s): "); 
        std::cin >> num_of_enu_target_;
        if(!x_target_.empty() || !y_target_.empty() || !z_target_.empty() || !yaw_target_.empty())
        {
            x_target_.clear();
            y_target_.clear();
            z_target_.clear();
            yaw_target_.clear();
        }
        for(int i=0; i<num_of_enu_target_; i++)
        {
            std::printf(" Target (%d) postion x, y, z (in meter): ", i+1);
            std::cin >> x >> y >> z;
            x_target_.push_back(x); 
            y_target_.push_back(y); 
            z_target_.push_back(z);
            std::printf(" Target (%d) yaw (in degree): ", i+1);
            std::cin >> yaw;
            yaw_target_.push_back(yaw);
            ros::spinOnce();
            rate.sleep();
        }
        std::printf(" Error to check target reached (in meter): ");
        std::cin >> target_error_;
    }
    else if(c == '2')
    {
        std::printf("[ INFO] Loaded prepared setpoints [x, y, z, yaw]\n");
        for(int i=0; i<num_of_enu_target_; i++)
        {
            std::printf(" Target (%d): [%.1f, %.1f, %.1f, %.1f]\n", i+1, x_target_[i], y_target_[i], z_target_[i], yaw_target_[i]);

            ros::spinOnce();
            rate.sleep();
        }
        std::printf(" Error to check target reached: %.1f (m)\n", target_error_);
    }
    else
    {
        inputENU();
    }
    enuFlight();
}
*/

/* perform flight with ENU (x,y,z) setpoints */
void OffboardControl::enuFlight()
{
    ros::Rate rate(10.0);
    int i=0;
    geometry_msgs::PoseStamped setpoint;
    std::printf("\n[ INFO] Target: [%.1f, %.1f, %.1f]\n", x_target_[i], y_target_[i], z_target_[i]);
    while(ros::ok())
    {
        if(i<(num_of_enu_target_-1))
        {
            final_position_reached_ = false;
            setpoint = targetTransfer(x_target_[i], y_target_[i], z_target_[i]);
        }
        else
        {
            final_position_reached_ = true;
            setpoint = targetTransfer(x_target_[num_of_enu_target_-1], y_target_[num_of_enu_target_-1], z_target_[num_of_enu_target_-1]);
        }

        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);

        distance_ = distanceBetween(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);
        std::printf("Distance to target: %.1f (m) \n", distance_);

        bool target_reached = checkPositionError(target_error_, setpoint);

        if(target_reached && !final_position_reached_)
        {
            std::printf("\n[ INFO] Reached position: [%.1f, %.1f, %.1f]\n", current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z); 
            
            hovering(setpoint, hover_time_);
            if(delivery_mode_enable_)
            {
                delivery(setpoint, unpack_time_);
            }
            std::printf("\n[ INFO] Next target: [%.1f, %.1f, %.1f]\n", x_target_[i+1], y_target_[i+1], z_target_[i+1]);
            i+=1;
        }
        if(target_reached && final_position_reached_)
        {
            std::printf("\n[ INFO] Reached Final position: [%.1f, %.1f, %.1f]\n", current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z); 
            hovering(setpoint, hover_time_);
            if(!return_home_mode_enable_)
            {
                landing(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0.0));
            }
            else 
            {
                if(delivery_mode_enable_) 
                {
                    delivery(setpoint, unpack_time_);
                }
                std::printf("\n[ INFO] Returning home [%.1f, %.1f, %.1f]\n",home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, home_enu_pose_.pose.position.z);
                returnHome(targetTransfer(home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, setpoint.pose.position.z));
                landing(home_enu_pose_);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

/* manage input for GPS setpoint flight mode: manual input from keyboard, load setpoints */
void OffboardControl::inputGPS()
{
    char c;
    std::printf("\n[ INFO] Please choose input method:\n");
    std::printf("- Choose 1: Manual enter from keyboard\n");
    std::printf("- Choose 2: Load prepared from launch file\n");
    std::printf("(1/2): ");
    std::cin >> c;
    if(c == '1')
    {
        double lat, lon, alt;
        std::printf("[ INFO] Manual enter GPS goal position(s)\n");
        std::printf(" Number of goal(s): "); 
        std::cin >> num_of_gps_goal_;
        if(!lat_goal_.empty() || !lon_goal_.empty() || !alt_goal_.empty())
        {
            lat_goal_.clear();
            lon_goal_.clear();
            alt_goal_.clear();
        }
        for(int i=0; i<num_of_gps_goal_; i++)
        {
            std::printf(" Goal (%d) postion Latitude (degree), Longitude (degree), Altitude (meter): ", i+1);
            std::cin >> lat >> lon >> alt;
            alt += current_gps_position_.altitude; 
            lat_goal_.push_back(lat); 
            lon_goal_.push_back(lon); 
            alt_goal_.push_back(alt);
        }
        std::printf(" Error to check goal reached (in meter): ");
        std::cin >> goal_error_;
    }
    else if(c == '2')
    {
        std::printf("[ INFO] Loaded prepared setpoints\n");
        for(int i=0; i<num_of_gps_goal_; i++)
        {
            alt_goal_[i] += current_gps_position_.altitude;
            std::printf(" Goal (%d): [%.8f, %.8f, %.3f]\n", i+1, lat_goal_[i], lon_goal_[i], alt_goal_[i]);
        }
        std::printf(" Error to check goal reached: %.1f (m)\n", goal_error_);
    }
    else
    {
        inputGPS();
    }
    waitForStable(10.0);
    setOffboardStream(10.0, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_));
    waitForArmAndOffboard(10.0);
    takeOff(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_), takeoff_hover_time_);
    std::printf("\n[ INFO] Flight with GPS setpoint\n");
    gpsFlight();
}

/* perform flight with GPS (LLA) setpoints
   check position when reached setpoint by (x,y,z) */
/*
void OffboardControl::gpsFlight()
{
    geometry_msgs::Point goal_enu;
    geometry_msgs::PoseStamped setpoint;
    ros::Rate rate(10.0);
    int i=0;
    std::printf("\n[ INFO] Goal: [%.8f, %.8f, %.3f]\n", lat_goal_[i], lon_goal_[i], alt_goal_[i]);
    while(ros::ok())
    {
        if(i<num_of_gps_goal_-1)
        {
            final_position_reached_ = false;
            goal_enu = WGS84ToENU(goalTransfer(lat_goal_[i], lon_goal_[i], alt_goal_[i]), ref_gps_position_);
        }
        else
        {
            final_position_reached_ = true;
            goal_enu = WGS84ToENU(goalTransfer(lat_goal_[num_of_gps_goal_-1], lon_goal_[num_of_gps_goal_-1], alt_goal_[num_of_gps_goal_-1]), ref_gps_position_);
        }
        
        setpoint = targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_);
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);

        distance_ = distanceBetween(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);
        std::printf("Distance to goal: %.1f (m) \n", distance_);

        bool goal_reached = checkPositionError(goal_error_, setpoint);

        if(goal_reached && !final_position_reached_)
        {
            std::printf("\n[ INFO] Reached position: [%.8f, %.8f, %.3f]\n", current_gps_position_.latitude, current_gps_position_.longitude, current_gps_position_.altitude); 
            
            hovering(setpoint, hover_time_);
            if(delivery_mode_enable_)
            {
                delivery(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_), unpack_time_);
            }
            std::printf("\n[ INFO] Next goal: [%.8f, %.8f, %.3f]\n", lat_goal_[i+1], lon_goal_[i+1], alt_goal_[i+1]);
            i+=1;
        }
        if(goal_reached && final_position_reached_)
        {
            std::printf("\n[ INFO] Reached Final position: [%.8f, %.8f, %.3f]\n", current_gps_position_.latitude, current_gps_position_.longitude, current_gps_position_.altitude); 
            hovering(setpoint, hover_time_);
            if(!return_home_mode_enable_)
            {
                landing(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0.0));
            }
            else
            {
                if(delivery_mode_enable_)
                {
                    delivery(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_), unpack_time_);
                }
                std::printf("\n[ INFO] Returning home [%.8f, %.8f, %.3f]\n",home_gps_position_.latitude, home_gps_position_.longitude, home_gps_position_.altitude);
                returnHome(targetTransfer(home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, setpoint.pose.position.z));
                landing(home_enu_pose_);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}
*/

/* perform flight with GPS (LLA) setpoints
   check position when reached setpoint by LLA */
void OffboardControl::gpsFlight()
{
    geometry_msgs::Point goal_enu;
    geometry_msgs::PoseStamped setpoint;
    ros::Rate rate(10.0);
    int i=0;
    std::printf("\n[ INFO] Goal: [%.8f, %.8f, %.3f]\n", lat_goal_[i], lon_goal_[i], alt_goal_[i]);
    while(ros::ok())
    {
        if(i<num_of_gps_goal_-1)
        {
            final_position_reached_ = false;
            goal_enu = WGS84ToENU(goalTransfer(lat_goal_[i], lon_goal_[i], alt_goal_[i]), ref_gps_position_);
        }
        else
        {
            final_position_reached_ = true;
            goal_enu = WGS84ToENU(goalTransfer(lat_goal_[num_of_gps_goal_-1], lon_goal_[num_of_gps_goal_-1], alt_goal_[num_of_gps_goal_-1]), ref_gps_position_);
        }
        
        setpoint = targetTransfer(goal_enu.x + x_offset_, goal_enu.y + y_offset_, goal_enu.z + z_offset_);
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);

        distance_ = distanceBetween(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);
        std::printf("Distance to goal: %.1f (m) \n", distance_);

        bool goal_reached = checkGPSError(goal_error_, goalTransfer(ENUToWGS84(current_odom_.pose.pose.position, ref_gps_position_).latitude,ENUToWGS84(current_odom_.pose.pose.position, ref_gps_position_).longitude, ENUToWGS84(current_odom_.pose.pose.position, ref_gps_position_).altitude), goalTransfer(lat_goal_[i], lon_goal_[i], alt_goal_[i]));

        if(goal_reached && !final_position_reached_)
        {
            std::printf("\n[ INFO] Reached position: [%.8f, %.8f, %.3f]\n", current_gps_position_.latitude, current_gps_position_.longitude, current_gps_position_.altitude); 
            
            hovering(setpoint, hover_time_);
            if(delivery_mode_enable_)
            {
                delivery(setpoint, unpack_time_);
            }
            std::printf("\n[ INFO] Next goal: [%.8f, %.8f, %.3f]\n", lat_goal_[i+1], lon_goal_[i+1], alt_goal_[i+1]);
            i+=1;
        }
        if(goal_reached && final_position_reached_)
        {
            std::printf("\n[ INFO] Reached Final position: [%.8f, %.8f, %.3f]\n", current_gps_position_.latitude, current_gps_position_.longitude, current_gps_position_.altitude); 
            hovering(setpoint, hover_time_);
            if(!return_home_mode_enable_)
            {
                landing(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0.0));
            }
            else
            {
                if(delivery_mode_enable_)
                {
                    delivery(setpoint, unpack_time_);
                }
                std::printf("\n[ INFO] Returning home [%.8f, %.8f, %.3f]\n",home_gps_position_.latitude, home_gps_position_.longitude, home_gps_position_.altitude);
                returnHome(targetTransfer(home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, setpoint.pose.position.z));
                landing(home_enu_pose_);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

/* manage for flight with optimization point from planner */
void OffboardControl::inputPlanner()
{
    ros::Rate rate(10.0);
    std::printf("[ INFO] Loaded global path setpoints\n");
    for(int i=0; i<num_of_enu_target_; i++)
    {
        std::printf(" Target (%d): [%.1f, %.1f, %.1f]\n", i+1, x_target_[i], y_target_[i], z_target_[i]);

        ros::spinOnce();
        rate.sleep();
    }
    waitForStable(10.0);
    setOffboardStream(10.0,targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_));
    waitForArmAndOffboard(10.0);
    takeOff(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, z_takeoff_), takeoff_hover_time_);
    std::printf("\n[ INFO] Flight with Planner setpoint\n");
    std::printf("\n[ INFO] Flight to start point of Optimization path\n");
    plannerFlight();
}

/* perform flight with ENU (x,y,z) setpoints from optimization planner 
   use yaw setpoint from calculate between current position and next optimization point */
void OffboardControl::plannerFlight()
{
    bool first_target_reached = false;
    bool final_reached = false;
    geometry_msgs::PoseStamped setpoint;
    ros::Rate rate(50.0);
    while(ros::ok())
    {
        setpoint = targetTransfer(x_target_[0], y_target_[0], z_target_[0]);
     
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);
     
        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x+components_vel_.x, current_odom_.pose.pose.position.y+components_vel_.y, current_odom_.pose.pose.position.z+components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);
        first_target_reached = checkPositionError(target_error_, setpoint);
        if(first_target_reached)
        {
            std::printf("\n[ INFO] Reached start point of Optimization path\n");
            hovering(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), 0.5);
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ros::Time current_time = ros::Time::now();
    if(opt_point_received_)
    {
        std::printf("[ INFO] Fly with optimization points\n");
        double last_alpha, curr_alpha, delta_alpha;
        while(ros::ok())
        {
            setpoint = targetTransfer(x_target_[num_of_enu_target_-1], y_target_[num_of_enu_target_-1], z_target_[num_of_enu_target_-1]);
            
            curr_alpha = calculateYawOffset(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), targetTransfer(opt_point_.x, opt_point_.y, opt_point_.z));

            target_enu_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(curr_alpha);

            target_enu_pose_.pose.position.x = opt_point_.x; 
            target_enu_pose_.pose.position.y = opt_point_.y; 
            target_enu_pose_.pose.position.z = opt_point_.z; 

            target_enu_pose_.header.stamp = ros::Time::now();
            setpoint_pose_pub_.publish(target_enu_pose_);
            
            final_reached = checkPositionError(target_error_, setpoint);
            
            if(final_reached)
            {
                std::printf("\n[ INFO] Reached Final position: [%.1f, %.1f, %.1f]\n", current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z); 
                hovering(setpoint, hover_time_);
                if(!return_home_mode_enable_)
                {
                    landing(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0.0));
                }
                else
                {
                    if(delivery_mode_enable_)
                    {
                        delivery(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_), unpack_time_);
                    }
                    std::printf("\n[ INFO] Returning home [%.1f, %.1f, %.1f]\n",home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, home_enu_pose_.pose.position.z);
                    returnHome(targetTransfer(home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, setpoint.pose.position.z));
                    landing(home_enu_pose_);
                }
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
    else
    {
        std::printf("\n[ WARN] Not received optimization points! Landing\n");
        landing(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, 0.0));
    }
}

/* perform flight with ENU (x,y,z) setpoints from optimization planner 
   not use yaw */
/*
void OffboardControl::plannerFlight()
{
    bool first_target_reached = false;
    bool target_reached = false;
    bool final_reached = false;
    geometry_msgs::PoseStamped setpoint;
    ros::Rate rate(50.0);
    while(ros::ok())
    {
        setpoint = targetTransfer(x_target_[0], y_target_[0], z_target_[0]);
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);
        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x+components_vel_.x, current_odom_.pose.pose.position.y+components_vel_.y, current_odom_.pose.pose.position.z+components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);
        first_target_reached = checkPositionError(target_error_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);
        if(first_target_reached)
        {
            std::printf("\n[ INFO] Reached start point of Optimization path\n");
            hovering(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), 0.5);
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
    ros::Time current_time = ros::Time::now();
    if(opt_point_received_)
    {
        std::printf("[ INFO] Fly with optimization points\n");
        double last_alpha = 0.0;
        double curr_alpha, delta_alpha;
        while(ros::ok())
        {
            setpoint = targetTransfer(x_target_[num_of_enu_target_-1], y_target_[num_of_enu_target_-1], z_target_[num_of_enu_target_-1]);
            
            curr_alpha = calculateYawOffset(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), targetTransfer(optimization_point_.front().x, optimization_point_.front().y, optimization_point_.front().z));
            delta_alpha = yaw_ - curr_alpha;

            target_enu_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_ + delta_alpha);

            target_enu_pose_.pose.position.x = optimization_point_.front().x;
            target_enu_pose_.pose.position.y = optimization_point_.front().y;
            target_enu_pose_.pose.position.z = optimization_point_.front().z;

            target_enu_pose_.header.stamp = ros::Time::now();
            setpoint_pose_pub_.publish(target_enu_pose_);
            
            target_reached = checkPositionError(0.1, targetTransfer(optimization_point_.front().x, optimization_point_.front().y, optimization_point_.front().z));

            final_reached = checkPositionError(target_error_, setpoint);
            
            if(target_reached && !final_reached)
            {
                optimization_point_.erase(optimization_point_.begin());
            }
            else if(target_reached && final_reached)
            {
                std::printf("\n[ INFO] Reached Final position: [%.1f, %.1f, %.1f]\n", current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z); 
                hovering(setpoint, hover_time_);
                if(!return_home_mode_enable_)
                {
                    landing(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0.0));
                }
                else
                {
                    if(delivery_mode_enable_)
                    {
                        delivery(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_), unpack_time_);
                    }
                    std::printf("\n[ INFO] Returning home [%.1f, %.1f, %.1f]\n",home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, home_enu_pose_.pose.position.z);
                    returnHome(targetTransfer(home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, setpoint.pose.position.z));
                    landing(home_enu_pose_);
                }
                break;
            }
            else
            {
                ros::spinOnce();
                rate.sleep();
            }
        }
    }
    else
    {
        std::printf("\n[ WARN] Not received optimization points! Landing\n");
        landing(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, 0.0));
    }
}
*/

/* perform flight with ENU (x,y,z) setpoints from optimization planner 
  testing with mav_trajectory_generation */
/*
void OffboardControl::plannerFlight()
{
    const bool oneshot = false;
    const bool autostart = false;
    max_a_ = 1.0;
    max_v_ = 1.0;
    dimension_ = 4;
    mav_trajectory_generation::Vertex::Vector vertices;
	mav_trajectory_generation::Vertex start(dimension_), middle(dimension_), end(dimension_); 
    std::vector<double> segment_times;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N_> non_opt(dimension_, parameters_);
    mav_trajectory_generation::Segment::Vector segments;
    visualization_msgs::MarkerArray markers;
    double distance = 0.5;
	mav_msgs::EigenTrajectoryPoint::Vector trajectory_point;

    start.makeStartOrEnd(Eigen::Vector4d(x_target_[0], y_target_[0], z_target_[0], yaw_target_[0]), derivative_to_optimize_);
    vertices.push_back(start);

    for(int i=1; i<num_of_enu_target_-1;i++)
    {
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector4d(x_target_[i], y_target_[i], z_target_[i], yaw_target_[i]));
        vertices.push_back(middle);
    }

    end.makeStartOrEnd(Eigen::Vector4d(x_target_[num_of_enu_target_-1], y_target_[num_of_enu_target_-1], z_target_[num_of_enu_target_-1], yaw_target_[num_of_enu_target_-1]), derivative_to_optimize_);
    vertices.push_back(end);

    if(vertices.size() == 1)
	{
		ROS_ERROR("Can't generate trajectory with only one point!\n");
		ros::shutdown();
	}
	else
	{
		ROS_INFO_STREAM_ONCE("\n" << vertices);
	}

	segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
    std::printf("[ generator] Used nonlinear optimization\n");
	non_opt.setupFromVertices(vertices, segment_times, derivative_to_optimize_);
	non_opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
	non_opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
	non_opt.optimize();
	non_opt.getPolynomialOptimizationRef().getSegments(&segments);
	non_opt.getTrajectory(&trajectory_);
    mav_trajectory_generation::drawMavTrajectory(trajectory_, distance, "map", &markers);
	marker_pub.publish(markers);
	
	current_sample_time_ = 0.0;
	start_time_ = ros::Time::now();
    while(ros::ok())
    {
        if(current_sample_time_ <= trajectory_.getMaxTime())
        {
            trajectory_msgs::MultiDOFJointTrajectory msg;
            mav_msgs::EigenTrajectoryPoint trajectory_point;
            bool success = mav_trajectory_generation::sampleTrajectoryAtTime(trajectory_, current_sample_time_, &trajectory_point);
            if(!success)
            {
                break;
            }
            mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
            msg.points[0].time_from_start = ros::Duration(current_sample_time_);
            msg.header.stamp = ros::Time::now();
            current_sample_time_ += dt_;

            // traj_gen::FlatTarget traj_msg;
            controller_msgs::FlatTarget traj_msg;
            traj_msg.type_mask = 2;
            traj_msg.header = msg.header;
            traj_msg.position = msg.points[0].transforms[0].translation;
            traj_msg.velocity = msg.points[0].velocities[0].linear;
            traj_msg.acceleration = msg.points[0].accelerations[0].linear;
            geometry_msgs::Quaternion quat = msg.points[0].transforms[0].rotation;
            double yaw = tf::getYaw(quat);
            std_msgs::Float32 yaw_msg;
            yaw_msg.data = static_cast<float>(yaw);

            flat_ref_pub_.publish(traj_msg);
            yaw_ref_pub_.publish(yaw_msg);
        }
        else
        {
            std::printf("\n\n\n[ INFO] Publish trajectory done\n");
            break;
        }
        ros::spinOnce();
    }



    if(opt_point_received_)
    {
        std::printf("[ INFO] Fly with optimization points\n");
        while(ros::ok())
        {
            setpoint = targetTransfer(x_target_[num_of_enu_target_-1], y_target_[num_of_enu_target_-1], z_target_[num_of_enu_target_-1]);
            // target_enu_pose_ = targetTransfer(optimization_point_.front().x, optimization_point_.front().y, optimization_point_.front().z);
            target_enu_pose_ = targetTransfer(opt_point_.x, opt_point_.y, opt_point_.z);
            target_enu_pose_.header.stamp = ros::Time::now();
            setpoint_pose_pub_.publish(target_enu_pose_);
            
            // if(target_reached && !final_reached)
            // {
                // optimization_point_.erase(optimization_point_.begin());
            // }
            // if(target_reached && final_reached)
            if(final_reached)
            {
                hovering(setpoint, hover_time_);
                if(!return_home_mode_enable_)
                {
                    landing(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0.0));
                }
                else
                {
                    if(delivery_mode_enable_)
                    {
                        delivery(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_), unpack_time_);
                    }
                    std::printf("\n[ INFO] Returning home [%.1f, %.1f, %.1f]\n",home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, home_enu_pose_.pose.position.z);
                    returnHome(targetTransfer(home_enu_pose_.pose.position.x, home_enu_pose_.pose.position.y, setpoint.pose.position.z));
                    landing(home_enu_pose_);
                }
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
    else
    {
        std::printf("\n[ WARN] Not received optimization points! Landing\n");
    }
}
*/

/* calculate yaw offset between current position and next optimization position */
double OffboardControl::calculateYawOffset(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped setpoint)
{
    double alpha;
    double xc = current.pose.position.x;
    double yc = current.pose.position.y;
    double xs = setpoint.pose.position.x;
    double ys = setpoint.pose.position.y;
    
    alpha = atan2(ys-yc, xs-xc);
    // std::cout << "\n[Debug] delta yaw " << alpha << "\n";
    // std::cout << "\n[Debug] delta yaw " << degreeOf(alpha) << "\n";
    return alpha;
}

/* perform takeoff task
   input: setpoint to takeoff and hover time */
void OffboardControl::takeOff(geometry_msgs::PoseStamped setpoint, double hover_time)
{
    ros::Rate rate(10.0);
    std::printf("\n[ INFO] Takeoff to [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
    bool takeoff_reached = false;
    while(ros::ok() && !takeoff_reached)
    {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);
        
        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);

        takeoff_reached = checkPositionError(target_error_, setpoint);
        if(takeoff_reached)
        {
            hovering(setpoint, hover_time);
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

/* perform hover task
   input: setpoint to hover and hover time */
void OffboardControl::hovering(geometry_msgs::PoseStamped setpoint, double hover_time)
{
    ros::Rate rate(10.0);
    ros::Time t_check;

    std::printf("\n[ INFO] Hovering at [%.1f, %.1f, %.1f] in %.1f (s)\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z, hover_time);
    t_check = ros::Time::now();
    while ((ros::Time::now() - t_check) < ros::Duration(hover_time))
    {
        setpoint_pose_pub_.publish(setpoint);

        ros::spinOnce();
    	rate.sleep();
    }
}

/* perform land task
   input: set point to land (e.g., [x, y, 0.0]) */
void OffboardControl::landing(geometry_msgs::PoseStamped setpoint)
{
    ros::Rate rate(10.0);
    bool land_reached = false;
    std::printf("[ INFO] Landing\n");
    while(ros::ok() && !land_reached)
    {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), setpoint);
        
        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);

        land_reached = checkPositionError(land_error_, setpoint);

        if(current_state_.system_status == 3)
        {
            std::printf("\n[ INFO] Land detected\n");
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if(set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent)
            {
                break;
            }
        }
        else if(land_reached)
        {
            flight_mode_.request.custom_mode = "AUTO.LAND";
            if(set_mode_client_.call(flight_mode_) && flight_mode_.response.mode_sent)
            {
                std::printf("\n[ INFO] LANDED\n");
            }
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
    
    operation_time_2_ = ros::Time::now();
    std::printf("\n[ INFO] Operation time %.1f (s)\n\n", (operation_time_2_-operation_time_1_).toSec());
    ros::shutdown();
}

/* perform return home task
   input: home pose in ENU (e.g., [home x, home y, 10.0])*/
void OffboardControl::returnHome(geometry_msgs::PoseStamped home_pose)
{
    ros::Rate rate(10.0);
    bool home_reached = false;
    while(ros::ok() && !home_reached)
    {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), home_pose);

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);

        home_reached = checkPositionError(target_error_, home_pose);
        if(home_reached)
        {
            hovering(home_pose, hover_time_);
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

/* perform delivery task
   input: current setpoint in trajectory and time to unpack */
void OffboardControl::delivery(geometry_msgs::PoseStamped setpoint, double unpack_time)
{
    ros::Rate rate(10.0);
    bool land_reached = false;
    std::printf("[ INFO] Land for unpacking\n");
    while(ros::ok() && !land_reached)
    {
        components_vel_ = velComponentsCalc(vel_desired_, targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_));

        target_enu_pose_ = targetTransfer(current_odom_.pose.pose.position.x + components_vel_.x, current_odom_.pose.pose.position.y + components_vel_.y, current_odom_.pose.pose.position.z + components_vel_.z);
        target_enu_pose_.header.stamp = ros::Time::now();
        setpoint_pose_pub_.publish(target_enu_pose_);
        
        if(current_state_.system_status == 3) 
        {
            land_reached = true;
        }
        else
        {
            land_reached = checkPositionError(land_error_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_));
        }

        if(land_reached)
        {
            if(current_state_.system_status == 3)
            {
                hovering(targetTransfer(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y, current_odom_.pose.pose.position.z), unpack_time);
                // TODO: unpack service
            }
            else
            {
                hovering(targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, z_delivery_), unpack_time);
                // TODO: unpack service
            }
            std::printf("\n[ INFO] Done! Return setpoint [%.1f, %.1f, %.1f]\n", setpoint.pose.position.x, setpoint.pose.position.y, setpoint.pose.position.z);
            returnHome(setpoint);
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

/* transfer lat, lon, alt setpoint to same message type with gps setpoint msg
   input: latitude, longitude and altitude that want to create sensor_msgs::NavSatFix msg */
sensor_msgs::NavSatFix OffboardControl::goalTransfer(double lat, double lon, double alt)
{
    sensor_msgs::NavSatFix goal;
    goal.latitude = lat;
    goal.longitude = lon;
    goal.altitude = alt;
    return goal;
}

/* transfer x, y, z setpoint to same message type with enu setpoint msg
   input: x, y, z that want to create geometry_msgs::PoseStamped msg */
geometry_msgs::PoseStamped OffboardControl::targetTransfer(double x, double y, double z)
{
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    return target;
}

/* transfer x, y, z (meter) and yaw (degree) setpoint to same message type with enu setpoint msg
   input: x, y, z in meter and yaw in degree that want to create geometry_msgs::PoseStamped msg */
geometry_msgs::PoseStamped OffboardControl::targetTransfer(double x, double y, double z, double yaw)
{
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    target.pose.orientation = tf::createQuaternionMsgFromYaw(radianOf(yaw));
    return target;
}

/* check offset between current position from odometry and setpoint position to decide when drone reached setpoint
   input: error in meter to check and target pose. This function check between current pose from odometry and target pose */
bool OffboardControl::checkPositionError(double error, geometry_msgs::PoseStamped target)
{
    Eigen::Vector3d geo_error;
    geo_error << target.pose.position.x - current_odom_.pose.pose.position.x, target.pose.position.y - current_odom_.pose.pose.position.y, target.pose.position.z - current_odom_.pose.pose.position.z;

	return (geo_error.norm() < error) ? true:false;
}

/* check offset between current position and setpoint position to decide when drone reached setpoint
   input: error in meter to check, current and target pose. This function check between current pose and target pose */
bool OffboardControl::checkPositionError(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
    Eigen::Vector3d geo_error;
    geo_error << target.pose.position.x - current.pose.position.x, target.pose.position.y - current.pose.position.y, target.pose.position.z - current.pose.position.z;

	return (geo_error.norm() < error) ? true:false;
}

/* check offset between current orientation and setpoint orientation to decide when drone reached setpoint
   input: error in degree to check, current pose and target pose */
bool OffboardControl::checkOrientationError(double error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
    Eigen::Vector3d current_rpy = getRPY(current.pose.orientation);
    Eigen::Vector3d target_rpy = getRPY(target.pose.orientation);
    return ((target_rpy-current_rpy).norm() < error) ? true:false;
}

/* check offset between current GPS and setpoint GPS to decide when drone reached setpoint
   input: error in meter to check altitude, current GPS and goal GPS postions */
bool OffboardControl::checkGPSError(double error, sensor_msgs::NavSatFix current, sensor_msgs::NavSatFix goal)
{
    // std::printf("\n[ Debug] Current [%f, %f, %f] Goal [%f, %f, %f]\n", current.latitude, current.longitude, current.altitude, goal.latitude, goal.longitude, goal.altitude);
    if((abs(current.latitude - goal.latitude)<=0.000001)&&(abs(current.longitude - goal.longitude)<=0.000001)&&(abs(current.altitude - goal.altitude)<=error))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* get roll, pitch and yaw angle from quaternion
   input: geometry_msgs::Quaternion */
Eigen::Vector3d OffboardControl::getRPY(geometry_msgs::Quaternion quat)
{
    tf::Quaternion q; 
    double r, p, y;
    Eigen::Vector3d rpy;

    tf::quaternionMsgToTF(quat, q);
    tf::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    rpy << r, p, y;
    return rpy;
}

/* create quaternion msg from roll, pitch and yaw
   input: roll, pitch and yaw angles */
/*
geometry_msgs::Quaternion OffboardControl::getQuaternionMsg(double roll, double pitch, double yaw)
{
    geometry_msgs::Quaternion msg;
    msg = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    return msg;
}
*/

/* calculate distance between current position and setpoint position
   input: current and target poses (ENU) to calculate distance */
double OffboardControl::distanceBetween(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
    Eigen::Vector3d distance;
    distance << target.pose.position.x - current.pose.position.x,
                target.pose.position.y - current.pose.position.y,
                target.pose.position.z - current.pose.position.z;

	return distance.norm();
}

/* calculate components of velocity about x, y, z axis
   input: desired velocity, current and target poses (ENU) */
geometry_msgs::Vector3 OffboardControl::velComponentsCalc(double v_desired, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
    double xc = current.pose.position.x;
    double yc = current.pose.position.y;
    double zc = current.pose.position.z;

    double xt = target.pose.position.x;
    double yt = target.pose.position.y;
    double zt = target.pose.position.z;

    double dx = xt - xc;
    double dy = yt - yc;
    double dz = zt - zc;

    double d = sqrt(sqr(dx) + sqr(dy) + sqr(dz));

    geometry_msgs::Vector3 vel;

    vel.x = ((dx/d) * v_desired);
    vel.y = ((dy/d) * v_desired);
    vel.z = ((dz/d) * v_desired);

    return vel;
}

/* convert from WGS84 GPS (LLA) to ECEF x,y,z
   input: GPS (LLA) in WGS84 (sensor_msgs::NavSatFix) */
geometry_msgs::Point OffboardControl::WGS84ToECEF(sensor_msgs::NavSatFix wgs84)
{
    geometry_msgs::Point ecef;
    double lambda = radianOf(wgs84.latitude);
    double phi = radianOf(wgs84.longitude);
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

/* convert from ECEF x,y,z to WGS84 GPS (LLA)
   input: point in ECEF */
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

    wgs84.latitude = degreeOf(phi);
    wgs84.longitude = degreeOf(lambda);

    return wgs84;
}

/* convert from ECEF x,y,z to ENU x,y,z
   input: point in ECEF and reference GPS */
geometry_msgs::Point OffboardControl::ECEFToENU(geometry_msgs::Point ecef, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point enu;
    double lambda = radianOf(ref.latitude);
    double phi = radianOf(ref.longitude);
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

/* convert from ENU x,y,z to ECEF x,y,z
   input: point in ENU and reference GPS */
geometry_msgs::Point OffboardControl::ENUToECEF(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point ecef;
    double lambda = radianOf(ref.latitude);
    double phi = radianOf(ref.longitude);
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

/* convert from WGS84 GPS (LLA) to ENU x,y,z
   input: GPS in WGS84 and reference GPS */
geometry_msgs::Point OffboardControl::WGS84ToENU(sensor_msgs::NavSatFix wgs84, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point ecef = WGS84ToECEF(wgs84);
    geometry_msgs::Point enu = ECEFToENU(ecef, ref);
    return enu;
}

/* convert from ENU x,y,z to WGS84 GPS (LLA)
   input: point in ENU and reference GPS */
geographic_msgs::GeoPoint OffboardControl::ENUToWGS84(geometry_msgs::Point enu, sensor_msgs::NavSatFix ref)
{
    geometry_msgs::Point ecef = ENUToECEF(enu, ref);
    geographic_msgs::GeoPoint wgs84 = ECEFToWGS84(ecef);
    return wgs84;
}