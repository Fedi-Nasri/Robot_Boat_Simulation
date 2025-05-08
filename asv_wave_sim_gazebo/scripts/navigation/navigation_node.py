#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from firebase_admin import db, credentials
import firebase_admin
import json
import time

class FirebaseNavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node')
        
        # Get parameters
        self.robot_id = rospy.get_param('~robot_id', 'default_robot')
        self.firebase_url = rospy.get_param('~firebase_url', 'https://oceancleaner-741db-default-rtdb.firebaseio.com')
        self.firebase_cred_path = rospy.get_param('~firebase_cred', '/home/fedi/asv_ws/credentials/oceancleaner-741db-firebase-adminsdk-fbsvc-776d874981.json')
        
        # Initialize Firebase
        self.init_firebase()
        
        # MoveBase client
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        
        # Main loop
        self.navigation_loop()

    def init_firebase(self):
        """Initialize Firebase connection"""
        try:
            cred = credentials.Certificate(self.firebase_cred_path)
            firebase_admin.initialize_app(cred, {
                'databaseURL': self.firebase_url
            })
            rospy.loginfo("Firebase initialized successfully")
        except Exception as e:
            rospy.logerr(f"Failed to initialize Firebase: {str(e)}")
            raise

    def get_navigation_points(self):
        """Retrieve navigation points from Firebase"""
        ref_path = f'navigation/coverage_path_planning/-OPm38xF9Gi3hppOoN8k/waypoints'
        try:
            ref = db.reference(ref_path)
            points_data = ref.get()
            
            if not points_data:
                rospy.logwarn("No navigation points found in Firebase")
                return None
            
            # Check if points_data is a list or dictionary
            points = []
            if isinstance(points_data, dict):
                for point_id, point_data in sorted(points_data.items(), key=lambda x: int(x[0])):
                    try:
                        x = float(point_data.get('x', 0))
                        y = float(point_data.get('y', 0))
                        points.append((x, y))
                    except (ValueError, AttributeError) as e:
                        rospy.logwarn(f"Invalid point data for {point_id}: {point_data}")
            elif isinstance(points_data, list):
                for idx, point_data in enumerate(points_data):
                    try:
                        x = float(point_data.get('x', 0))
                        y = float(point_data.get('y', 0))
                        points.append((x, y))
                    except (ValueError, AttributeError) as e:
                        rospy.logwarn(f"Invalid point data at index {idx}: {point_data}")
            else:
                rospy.logerr("Unexpected data format for navigation points")
                return None
            
            # Print the first 5 points
            rospy.loginfo(f"First 5 points loaded: {points[:5]}")
            return points if points else None
            
        except Exception as e:
            rospy.logerr(f"Error retrieving navigation points: {str(e)}")
            return None

    def send_goal(self, x, y):
        """Send navigation goal to move_base"""
       
        # Transform coordinates
        origin_offset_x = 75.0  # Changed from the yaml file
        origin_offset_y = 9.0   # Changed from the yaml file
        x_transformed = x - origin_offset_x
        y_transformed = y - origin_offset_y
       
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set position (assuming fixed orientation for simplicity)
        goal.target_pose.pose.position = Point(x_transformed, y_transformed, 0)
        goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)  # Facing forward
        
        self.move_base_client.send_goal(goal)
        rospy.loginfo(f"Sent goal to position ({x_transformed}, {y_transformed})")
        
        # Wait for result with timeout
        wait = self.move_base_client.wait_for_result(rospy.Duration(60))
        
        if not wait:
            rospy.logwarn("Timed out waiting for move_base result")
            self.move_base_client.cancel_goal()
            return False
        
        return self.move_base_client.get_result()

    def navigation_loop(self):
        """Main navigation loop"""
        while not rospy.is_shutdown():
            points = self.get_navigation_points()
            
            if not points:
                rospy.loginfo("No points to navigate to. Waiting...")
                rospy.sleep(5)
                continue
                
            for idx, (x, y) in enumerate(points, 1):
                if rospy.is_shutdown():
                    return
                    
                rospy.loginfo(f"Navigating to point {idx}/{len(points)}: ({x}, {y})")
                
                result = self.send_goal(x, y)
                
                if result:
                    rospy.loginfo(f"Successfully reached point {idx}")
                else:
                    rospy.logwarn(f"Failed to reach point {idx}")
                    # Optionally: add retry logic here
                
                # Print the current point being processed
                rospy.loginfo(f"Current point being processed: ({x}, {y})")
                
                # Small delay between points
                rospy.sleep(1)
            
            rospy.loginfo("Completed all navigation points. Waiting for new points...")
            rospy.sleep(10)  # Wait before checking for new points again

if __name__ == '__main__':
    try:
        node = FirebaseNavigationNode()
    except rospy.ROSInterruptException:
        pass