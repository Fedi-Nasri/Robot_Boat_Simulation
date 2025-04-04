#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from gazebo_msgs.srv import BodyRequest

class FanController:
    def __init__(self):
        rospy.init_node('fan_controller')
        
        # Subscribers for each fan's torque command
        rospy.Subscriber('/fan_droit', Float64, self.droit_callback)
        rospy.Subscriber('/fan_gauche', Float64, self.gauche_callback)
        
        # Initialize Gazebo services
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        rospy.wait_for_service('/gazebo/clear_body_wrenches')
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.clear_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
        
        # Initialize with zero torque
        self.current_droit = 0.0
        self.current_gauche = 0.0
        self.apply_torque("boatcleaningc::fandroit", 0.0)
        self.apply_torque("boatcleaningc::fangauche", 0.0)
        
        rospy.loginfo("Fan controller ready")
        rospy.loginfo("Publish Float64 to:")
        rospy.loginfo("- /fan_droit for right fan torque")
        rospy.loginfo("- /fan_gauche for left fan torque")

    def droit_callback(self, msg):
        """Handle right fan torque commands"""
        self.current_droit = msg.data
        self.apply_torque("boatcleaningc::fandroit", self.current_droit)
        rospy.loginfo(f"Right fan torque set to: {self.current_droit} Nm")

    def gauche_callback(self, msg):
        """Handle left fan torque commands"""
        self.current_gauche = msg.data
        self.apply_torque("boatcleaningc::fangauche", self.current_gauche)
        rospy.loginfo(f"Left fan torque set to: {self.current_gauche} Nm")

    def apply_torque(self, link_name, torque):
        """Apply torque to a specific fan"""
        try:
            # Clear existing wrench
            self.clear_wrench(link_name)
            
            # Apply new torque if non-zero
            if abs(torque) > 0.001:
                req = ApplyBodyWrenchRequest()
                req.body_name = link_name
                req.reference_frame = "world"
                req.wrench.torque.x = torque
                req.duration = rospy.Duration(-1)  # Continuous
                self.apply_wrench(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"Torque application failed for {link_name}: {str(e)}")

if __name__ == '__main__':
    controller = FanController()
    rospy.spin()