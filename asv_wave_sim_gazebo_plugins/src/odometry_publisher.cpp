#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>

// Class to manage odometry publishing
class OdometryPublisher {
public:
    // Constructor initializes ROS objects after ros::init()
    OdometryPublisher(ros::NodeHandle& nh) {
        odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
        link_states_sub = nh.subscribe("/gazebo/link_states", 10, &OdometryPublisher::linkStatesCallback, this);
    }

    // Callback to process Gazebo link states
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

        // Use current ROS time since msg->header.stamp is not available
        ros::Time current_time = ros::Time::now();

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

private:
    ros::Publisher odom_pub;                   // Publisher for odometry messages
    ros::Subscriber link_states_sub;           // Subscriber for Gazebo link states
    tf::TransformBroadcaster odom_broadcaster; // Broadcaster for tf transforms
};

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh;

    // Create an instance of the OdometryPublisher class
    OdometryPublisher odom_publisher(nh);

    // Check if simulation time is being used
    if (!ros::Time::isSimTime()) {
        ROS_WARN("ROS is not using simulation time. Set '/use_sim_time' to true.");
    }

    // Spin to process callbacks
    ros::spin();
    return 0;
}

