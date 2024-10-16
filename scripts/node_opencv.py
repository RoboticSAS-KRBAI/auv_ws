#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from std_msgs.msg import Bool, Int8

flag = 0

def detect_color(frame, lower_hue, upper_hue, lower_saturation, upper_saturation, lower_value, upper_value):
    # Convert frame from BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define color range in HSV
    lower_bound = np.array([lower_hue, lower_saturation, lower_value])
    upper_bound = np.array([upper_hue, upper_saturation, upper_value])
    
    # Masking color based on the defined range
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # Display mask result
    result = cv2.bitwise_and(frame, frame, mask=mask)
    
    return result, mask

def callback_flag(data: Int8):
    global flag
    flag = data.data

def main():
    # Open camera
    cap = cv2.VideoCapture(0)  # Using the primary camera (index 0)
    # Set the frame width and height
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    center_x = 0
    center_y = 0
    
    

    # Set original video frame rate
    # HSV values for color detection
    lower_hue = 150
    upper_hue = 179
    lower_saturation = 110
    upper_saturation = 255
    lower_value = 0
    upper_value = 255
    
    # Initialize ROS node
    rospy.init_node('color_detection_node')
    
    # Publisher for the boolean message
    pub = rospy.Publisher('/rosserial/bucket_detected', Bool, queue_size=10)
    rospy.Subscriber('/nuc/flag', Int8, callback_flag)

    while not rospy.is_shutdown():
        # Read frame from the camera
        ret, frame = cap.read()
        
        if not ret:
            break
        
        # Detect color based on HSV values
        color_detected_frame, mask = detect_color(frame, lower_hue, upper_hue, lower_saturation, upper_saturation, lower_value, upper_value)
        
        # Find contours from the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        bounding_box_area = 0

        if contours:
            # Find the largest contour based on area
            largest_contour = max(contours, key=cv2.contourArea)
            bounding_box_area = cv2.contourArea(largest_contour)
            
            # Create a bounding box around the object with the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Calculate the center point of the bounding box
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
            cv2.putText(frame, f"Object Center: ({center_x}, {center_y})", (center_x - 20, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Find the center point of the frame
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2
        cv2.circle(frame, (frame_center_x, frame_center_y), 5, (255, 0, 0), -1)
        cv2.putText(frame, f"Frame Center: ({frame_center_x}, {frame_center_y})", (frame_center_x + 10, frame_center_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Publish a boolean indicating whether the object is within 20 pixels of the frame center
        if abs(frame_center_x - center_x) <= 50 and abs(frame_center_y - center_y) <= 50 and bounding_box_area >= 180000 : #and flag == 3 :
            rospy.loginfo("Ball Dropping")
            object_detected = True
        else:
            object_detected = False
        pub.publish(object_detected)
        
        # Display the results
        # cv2.imshow('Original Video', frame)
        # cv2.imshow('Color Detection', color_detected_frame)
        
        # Break the loop by pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the video capture and close all windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.loginfo("Node OpenCV has been started")
    main()
