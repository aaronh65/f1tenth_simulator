#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

class StraightPlanner {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle, min_safe_distance, max_safe_distance;

	// state variables
	double curr_speed=0;

    // Listen for odom messages
    ros::Subscriber odom_sub;
	ros::Subscriber scan_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle=0.0;

	// basic collision safety
	bool too_close=false; 


public:
    StraightPlanner() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic, scan_topic;
        n.getParam("straight_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
		n.getParam("scan_topic", scan_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
		n.getParam("min_safe_distance", min_safe_distance);
		n.getParam("max_safe_distance", max_safe_distance);
		//ROS_INFO_STREAM("got min safe distance " << min_safe_distance);

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe(odom_topic, 1, &StraightPlanner::odom_callback, this);
		scan_sub = n.subscribe(scan_topic, 1, &StraightPlanner::scan_callback, this);


    }

    void odom_callback(const nav_msgs::Odometry & msg) {
        // publishing is done in odom callback just so it's at the same rate as the sim

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
		if (too_close) {
			drive_msg.speed = 0;
		} else {
        	drive_msg.speed = max_speed / 2.0;
		}
		curr_speed = drive_msg.speed;
	
        // set angle (add random change to previous angle)
        drive_msg.steering_angle = 0;

        // reset previous desired angle
        prev_angle = drive_msg.steering_angle;

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);


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
    ros::init(argc, argv, "straight_planner");
    StraightPlanner sp;
    ros::spin();
    return 0;
}
