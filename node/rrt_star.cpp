#include <ros/ros.h>

#include <f1tenth_simulator/rrt_star.h>

/////////////////
// constructor //
/////////////////

RRTstar::RRTstar(ros::NodeHandle &nh): nh_(nh){
        // Initialize the node handle
	buf = ros::Duration(0.01);

	// get topic names
	std::string drive_topic, odom_topic, scan_topic, goal_topic;
	nh_.getParam("rrt_star_drive_topic", drive_topic);
	nh_.getParam("odom_topic", odom_topic);
	nh_.getParam("scan_topic", scan_topic);
	nh_.getParam("goal_topic", goal_topic);

	// get car parameters
	nh_.getParam("max_speed", max_speed);
	nh_.getParam("max_steering_angle", max_steering_angle);
	nh_.getParam("goal_threshold", goal_threshold);

	// Make a publisher for drive messages
	drive_pub = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

	// Start a subscriber to listen to odom messages
	ROS_INFO_STREAM(odom_topic);
	odom_sub = nh_.subscribe(odom_topic, 1, &RRTstar::odom_callback, this);
	goal_sub = nh_.subscribe(goal_topic, 1, &RRTstar::goal_callback, this);

}

RRTstar::~RRTstar() {
	ROS_INFO("RRTstar shutting down");
}

///////////////
// callbacks //
///////////////

void RRTstar::odom_callback(const nav_msgs::Odometry & msg) {
	// publishing is done in odom callback just so it's at the same rate as the sim

	ROS_INFO_STREAM("running rrt");
}


void RRTstar::goal_callback(const geometry_msgs::PoseStamped & pose_stamped) {
	goal = pose_stamped;
	ROS_INFO_STREAM("new goal specified at " << 
			goal.pose.position.x << ", " << 
			goal.pose.position.y);
	open_goal = true;		

	// start RRT
}

/////////////////
// RRT methods //
/////////////////

/////////////////
// main method //
/////////////////
int main(int argc, char ** argv) {
    ros::init(argc, argv, "rrt_star_planner");
	ros::NodeHandle nh = ros::NodeHandle("~"); 
    RRTstar rrt(nh);
    ros::spin();
    return 0;
}
