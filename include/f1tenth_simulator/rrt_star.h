#ifndef RRT_STAR
#define RRT_STAR

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <iostream>



typedef struct Node {
	double x, y;
	double cost;
	int parent;
	bool is_root = false;
} Node;


class RRTstar {
public:
	RRTstar(ros::NodeHandle &nh);
	//virtual ~RRT();
	void odom_callback(const nav_msgs::Odometry & msg);
	void goal_callback(const geometry_msgs::PoseStamped & pose_stamped);

private:
	// the ROS node
    ros::NodeHandle nh_;

    // car parameters, state variables
    double max_speed, max_steering_angle;
	double goal_threshold;
	
	// rrt params
	double steer_distance, goal_sample_rate;
	bool open_goal=false;

    // subscribers + publishers
    ros::Subscriber odom_sub;
	ros::Subscriber scan_sub;
	ros::Subscriber goal_sub;

    ros::Publisher drive_pub;
	ros::Duration buf;

    // previous desired steering angle
    double prev_angle=0.0;

	// basic collision safety
	bool too_close=false; 

	// goal
	geometry_msgs::PoseStamped goal;
	geometry_msgs::PoseStamped pose;

	// transforms
	tf::TransformListener listener;


};


#endif // RRT_STAR
