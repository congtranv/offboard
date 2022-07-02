#ifndef TRAJ_GEN_H_
#define TRAJ_GEN_H_

#include<mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include<mav_trajectory_generation/trajectory.h>
#include<mav_trajectory_generation_ros/ros_visualization.h>
#include<mav_trajectory_generation_ros/ros_conversions.h>
#include<ros/ros.h>
#include<visualization_msgs/Marker.h> 
#include<visualization_msgs/MarkerArray.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<tf/transform_datatypes.h>
#include<eigen_conversions/eigen_msg.h>
// #include<offboard/FlatTarget.h>
#include<std_msgs/Float32.h>
#include<vector>

double max_v_;
double max_a_;
double max_ang_v_;
double max_ang_a_;
double yaw_;
double init_z_;

ros::Timer publish_timer_;
ros::Publisher flat_ref_pub_;
ros::Publisher yaw_ref_pub_;
ros::Time start_time_;
double current_sample_time_;

Eigen::Vector3d start_pos_;
Eigen::Vector3d target_pos_;
Eigen::Vector3d middle_pos_;
Eigen::Vector3d init_pos_;
Eigen::Vector3d target_vel_;

bool odom_received_ = false;
bool input_middle_ = false;

Eigen::Affine3d current_pose_ = Eigen::Affine3d::Identity();
Eigen::Affine3d ref_pose_se3_ = Eigen::Affine3d::Identity();
Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();
Eigen::Vector3d current_ang_vel_ = Eigen::Vector3d::Zero();

double dt_ = 0.01;
int dimension_;
const int N_ = 10;
bool nonlinear_ = true;
const int derivative_to_optimize_ = mav_trajectory_generation::derivative_order::SNAP;
mav_trajectory_generation::NonlinearOptimizationParameters parameters_;
mav_trajectory_generation::Trajectory trajectory_;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void cmdTimerCallback(const ros::TimerEvent&);
void refPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

#endif