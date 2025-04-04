#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from gazebo_msgs.srv import BodyRequest

class BoatController:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('boat_controller', anonymous=True)

        # Subscriber for keyboard input (from teleop_twist_keyboard)
        self.cmd_vel_sub = rospy.Subscriber('/boatcleaningc/cmd_vel', Twist, self.cmd_vel_callback)

        # Service clients for applying torques in Gazebo
        self.apply_wrench_srv = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.clear_wrench_srv = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)

        # Base torque magnitude (will be adjusted by up/down arrows)
        self.torque_magnitude = 0.0  # Start with 0 torque

        rospy.loginfo("BoatController node started.")
        rospy.loginfo("Controls: a (right motor), e (left motor), up arrow (increase torque), down arrow (decrease torque), x (stop)")

    def cmd_vel_callback(self, msg):
        # Extract angular speed (a/e keys) and linear speed (up/down arrows)
        angular_speed = msg.angular.z  # a (positive), e (negative)
        linear_speed = msg.linear.x    # up arrow (positive), down arrow (negative)

        # Update torque magnitude based on up/down arrows
        if linear_speed > 0.0:  # Up arrow pressed
            self.torque_magnitude += 0.1  # Increase torque by 0.1 Nm per press
        elif linear_speed < 0.0:  # Down arrow pressed
            self.torque_magnitude -= 0.1  # Decrease torque by 0.1 Nm per press
            self.torque_magnitude = max(0.0, self.torque_magnitude)  # Don't go below 0

        # Clear previous torques to avoid accumulation
        self.clear_wrenches()

        # Apply torque to fandroit (right motor) if a is pressed
        if angular_speed > 0.0 and self.torque_magnitude > 0.0:
            self.apply_torque("boatcleaningc::fandroit", self.torque_magnitude, 0.0, 0.0)
            rospy.loginfo("Right motor (fandroit) torque: %f Nm", self.torque_magnitude)

        # Apply torque to fangauche (left motor) if e is pressed
        if angular_speed < 0.0 and self.torque_magnitude > 0.0:
            self.apply_torque("boatcleaningc::fangauche", self.torque_magnitude, 0.0, 0.0)
            rospy.loginfo("Left motor (fangauche) torque: %f Nm", self.torque_magnitude)

        # If both motors are off and torque is 0, log the state
        if angular_speed == 0.0 and linear_speed == 0.0:
            self.torque_magnitude = 0.0  # Reset torque when stopped (x key)
            rospy.loginfo("Motors stopped, torque reset to 0")

    def apply_torque(self, link_name, tx, ty, tz):
        try:
            wrench = ApplyBodyWrenchRequest()
            wrench.body_name = link_name
            wrench.wrench.torque.x = tx
            wrench.wrench.torque.y = ty
            wrench.wrench.torque.z = tz
            wrench.duration = rospy.Duration(-1)  # Apply indefinitely until cleared
            self.apply_wrench_srv(wrench)
            rospy.loginfo("Applied torque to %s: (%f, %f, %f)", link_name, tx, ty, tz)
        except rospy.ServiceException as e:
            rospy.logerr("Failed to apply torque to %s: %s", link_name, str(e))

    def clear_wrenches(self):
        try:
            self.clear_wrench_srv("boatcleaningc::fandroit")
            self.clear_wrench_srv("boatcleaningc::fangauche")
        except rospy.ServiceException as e:
            rospy.logerr("Failed to clear wrenches: %s", str(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = BoatController()
        controller.run()
    except rospy.ROSInterruptException:
        pass