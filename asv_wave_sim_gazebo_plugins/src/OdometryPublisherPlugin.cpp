#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>
#include <rosgraph_msgs/Clock.h>

// Global variables
ros::Publisher odom_pub;
tf::TransformBroadcaster odom_broadcaster;
ros::Time current_time;  // Store the latest simulation time

void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg) {
  current_time = msg->clock;  // Update current_time with simulation time
}

void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {
  // Find the index of "boatcleaningc::baseboatclening"
  int index = -1;
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "boatcleaningc::baseboatclening") {
      index = i;
      break;
    }
  }
  if (index == -1) {
    ROS_WARN("Link 'boatcleaningc::baseboatclening' not found in /gazebo/link_states");
    return;
  }

  // Extract pose and twist from Gazebo
  geometry_msgs::Pose pose = msg->pose[index];
  geometry_msgs::Twist twist = msg->twist[index];

  // Publish transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "baseboatclening";
  odom_trans.transform.translation.x = pose.position.x;
  odom_trans.transform.translation.y = pose.position.y;
  odom_trans.transform.translation.z = pose.position.z;
  odom_trans.transform.rotation = pose.orientation;
  odom_broadcaster.sendTransform(odom_trans);

  // Publish odometry message
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "baseboatclening";
  odom.pose.pose = pose;
  odom.twist.twist = twist;
  odom_pub.publish(odom);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;

  // Initialize publisher and subscribers
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber link_states_sub = n.subscribe("/gazebo/link_states", 10, linkStatesCallback);
  ros::Subscriber clock_sub = n.subscribe("/clock", 1, clockCallback);

  // Check if simulation time is being used
  if (!ros::Time::isSimTime()) {
    ROS_WARN("ROS is not using simulation time. Set '/use_sim_time' to true.");
  }

  // Initialize current_time to avoid using uninitialized value
  current_time = ros::Time::now();

  // Spin to process callbacks
  ros::spin();
  return 0;
}
