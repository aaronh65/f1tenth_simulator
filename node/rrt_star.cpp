#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

// Transforms
#include <tf/transform_listener.h>

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

class RRTstar {

private:
    // the ROS node
    ros::NodeHandle n;

    // car parameters, state variables
    double max_speed, max_steering_angle, min_safe_distance, max_safe_distance;
	double curr_speed=0, pursuit_speed;
	double goal_threshold;
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

	// transforms
	tf::TransformListener listener;

public:
    RRTstar() {
        // Initialize the node handle
        n = ros::NodeHandle("~");
		buf = ros::Duration(0.01);

        // get topic names
        std::string drive_topic, odom_topic, scan_topic, goal_topic;
        n.getParam("pursuit_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
		n.getParam("scan_topic", scan_topic);
		n.getParam("goal_topic", goal_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
		n.getParam("min_safe_distance", min_safe_distance);
		n.getParam("max_safe_distance", max_safe_distance);
		n.getParam("goal_threshold", goal_threshold);
		n.getParam("pursuit_speed", pursuit_speed);
		//ROS_INFO_STREAM("got min safe distance " << min_safe_distance);

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe(odom_topic, 1, &PurePursuit::odom_callback, this);
		goal_sub = n.subscribe(goal_topic, 1, &PurePursuit::goal_callback, this);

    }


    void odom_callback(const nav_msgs::Odometry & msg) {
        // publishing is done in odom callback just so it's at the same rate as the sim

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;
		
		if (!open_goal) {
			drive_pub.publish(drive_st_msg);
			return;
		}

		// transform goal pose into ego frame
		goal.header.stamp = ros::Time::now()-buf;
		geometry_msgs::PoseStamped goal_ego;
		listener.transformPose("/base_link",
							   goal, // given in map frame
							   goal_ego);
		double l = pow(goal_ego.pose.position.x, 2) +
				   pow(goal_ego.pose.position.y, 2);
		l = pow(l, 0.5);
		// ROS_INFO_STREAM("distance to goal " << l);

		// task complete if we're within 0.6m of goal
		if (l < goal_threshold) { // parameterize this
			drive_pub.publish(drive_st_msg);
			open_goal = false;
			ROS_INFO_STREAM("reached goal");
			return;
		}

		double target_angle = 2*goal_ego.pose.position.y / pow(l, 2);
		drive_msg.speed = pursuit_speed;
		drive_msg.steering_angle = target_angle;
		prev_angle = target_angle;
		// ROS_INFO_STREAM("target angle is " << target_angle);

        // publish stamped drive message
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);
    }


	void goal_callback(const geometry_msgs::PoseStamped pose_stamped) {
		goal = pose_stamped;
		ROS_INFO_STREAM("new goal specified at " << 
				goal.pose.position.x << ", " << 
				goal.pose.position.y);
		open_goal = true;		
	}
	

	void scan_callback(const sensor_msgs::LaserScan& msg) {
		double min_front_scan = std::numeric_limits<double>::max();
		for (double range : msg.ranges) {
			if (range < min_front_scan && range < msg.range_max && range > msg.range_min) {
				min_front_scan = range;
			}
		}

		if (min_front_scan == std::numeric_limits<double>::max()) {
			ROS_WARN("no valid ranges in laser scan");
			return;
		}

		double threshold = std::max(min_safe_distance, std::min(max_safe_distance, curr_speed/2.0)); 
		if (min_front_scan < threshold) {
			if (!too_close) {
				//ROS_INFO("too close");
			}
			too_close = true;
		} else {
			if (too_close) {
				//ROS_INFO("back to safety");
			}
			too_close = false;
		}
	}

}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "rrt_star_planner");
    RRTstar rrt;
    ros::spin();
    return 0;
}
