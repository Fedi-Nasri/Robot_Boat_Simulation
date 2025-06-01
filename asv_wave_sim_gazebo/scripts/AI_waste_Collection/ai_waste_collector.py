#!/usr/bin/env python3

import rospy
from asv_wave_sim_gazebo.msg import WasteDetection
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest

class SimpleWasteCollector:
    def __init__(self):
        rospy.init_node('simple_waste_collector', anonymous=True)
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        rospy.wait_for_service('/gazebo/clear_body_wrenches')
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        self.clear_wrench = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
        self.max_torque = -1.0
        self.fan_right = "boatcleaningc::fandroit"
        self.fan_left = "boatcleaningc::fangauche"
        self.state = "idle"
        self.last_bottom_middle = 0
        rospy.Subscriber('/waste_detection', WasteDetection, self.waste_detection_callback)
        rospy.loginfo("Simple Waste Collector node started.")

    def apply_torque(self, link_name, torque):
        try:
            self.clear_wrench(link_name)
            if abs(torque) > 0.001:
                req = ApplyBodyWrenchRequest()
                req.body_name = link_name
                req.reference_frame = "world"
                req.wrench.torque.x = torque
                req.duration = rospy.Duration(-1)
                self.apply_wrench(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to apply torque on {link_name}: {e}")

    def stop_fans(self):
        self.apply_torque(self.fan_right, 0.0)
        self.apply_torque(self.fan_left, 0.0)
        self.state = "idle"
        rospy.loginfo("Boat stopped.")

    def move_forward(self):
        self.apply_torque(self.fan_right, self.max_torque)
        self.apply_torque(self.fan_left, self.max_torque)
        self.state = "moving"
        rospy.loginfo("Boat moving forward.")

    def rotate_left(self, msg=None):
        self.apply_torque(self.fan_right, 0.0)
        self.apply_torque(self.fan_left, self.max_torque)
        self.state = "rotating_left"
        rospy.loginfo("Boat rotating left.")
        rospy.sleep(0.5)
        self.stop_fans()
        if msg is not None:
            self.waste_detection_callback(msg)

    def rotate_right(self, msg=None):
        self.apply_torque(self.fan_right, self.max_torque )
        self.apply_torque(self.fan_left, 0.0)
        self.state = "rotating_right"
        rospy.loginfo("Boat rotating right.")
        rospy.sleep(0.5)
        self.stop_fans()
        if msg is not None:
            self.waste_detection_callback(msg)

    def is_waste_in_upper_middle(self, msg):
        for section in msg.sections:
            if section.section == "Upper Middle" and section.count >= 1:
                return True
        return False

    def waste_detection_callback(self, msg):
        upper_middle_count = 0
        bottom_middle_count = 0
        left_count = 0
        right_count = 0
        for section in msg.sections:
            if section.section == "Upper Middle":
                upper_middle_count = section.count
            elif section.section == "Bottom Middle":
                bottom_middle_count = section.count
            elif section.section == "Left":
                left_count = section.count
            elif section.section == "Right":
                right_count = section.count
        rospy.loginfo(f"Left: {left_count}, Right: {right_count}, Upper Middle: {upper_middle_count}, Bottom Middle: {bottom_middle_count}")
        # Priority: Move waste to Upper Middle, then forward, then stop at Bottom Middle
        if upper_middle_count >= 1:
            self.move_forward()
            if bottom_middle_count >= 1:
                self.stop_fans()
        elif right_count >= 1:
            self.rotate_right(msg)
        elif left_count >= 1:
            self.rotate_left(msg)
        else:
            self.stop_fans()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SimpleWasteCollector()
        node.run()
    except rospy.ROSInterruptException:
        pass
