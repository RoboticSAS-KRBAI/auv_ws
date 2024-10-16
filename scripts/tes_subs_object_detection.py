#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from robotic_sas_auv_ros.msg import ObjectDetection, BoundingBox

class ObjectDetectionSubscriber:
    def __init__(self):
        rospy.init_node('object_detection_subscriber', anonymous=True)
        
        # Publisher to publish x-coordinate differences
        self.pub_x_difference = rospy.Publisher('x_difference', Int16, queue_size=10)
        
        # Subscriber to subscribe to the object detection messages
        rospy.Subscriber('object_detection', ObjectDetection, self.object_detection_callback)
        
        # Spin to keep the script for listening
        rospy.spin()

    def object_detection_callback(self, msg):
        rospy.loginfo("Received object detection message with %d bounding boxes", len(msg.bounding_boxes))
        
        # Initialize variables to track the bounding box with the highest probability
        max_prob = -1
        max_prob_bbox = None

        # Center x-coordinate of the frame
        frame_center_x = 640 // 2
        
        # Iterate over all bounding boxes to find the one with the highest probability
        for bbox in msg.bounding_boxes:
            if bbox.probability > max_prob:
                max_prob = bbox.probability
                max_prob_bbox = bbox

        if max_prob_bbox:
            # Log the details of the bounding box with the highest probability
            rospy.loginfo("Bounding box with highest probability:")
            rospy.loginfo("Class: %s, Probability: %.2f, Coordinates: (%d, %d), (%d, %d)",
                          max_prob_bbox.class_name, max_prob_bbox.probability, 
                          max_prob_bbox.x_min, max_prob_bbox.y_min, 
                          max_prob_bbox.x_max, max_prob_bbox.y_max)
            
            # Calculate the center x-coordinate of the bounding box
            center_x = (max_prob_bbox.x_min + max_prob_bbox.x_max) // 2
            
            # Calculate the pixel difference between the center of the bounding box and the center of the frame
            x_difference = center_x - frame_center_x
            self.pub_x_difference.publish(x_difference)
            rospy.loginfo("Center x-coordinate of bounding box: %d", center_x)
            rospy.loginfo("Pixel difference between bounding box center and frame center: %d", x_difference)
        else:
            rospy.loginfo("No bounding boxes detected.")

if __name__ == '__main__':
    try:
        ObjectDetectionSubscriber()
    except rospy.ROSInterruptException:
        pass
