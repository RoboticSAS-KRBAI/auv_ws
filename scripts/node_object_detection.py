#!/usr/bin/env python3
import cv2
import time
from ultralytics import YOLO
import rospy
from robotic_sas_auv_ros.msg import BoundingBox, ObjectDetection

# Load the YOLOv8 model
model = YOLO('krbai_6.pt')

# Set CUDA if available
def get_cuda_device():
    if cv2.cuda.getCudaEnabledDeviceCount() > 0:
        return True
    return False

use_cuda = get_cuda_device()

# Initialize ROS node
rospy.init_node('node_object_detection', anonymous=True)
obj_det_pub = rospy.Publisher('/nuc/object_detection', ObjectDetection, queue_size=10)

# Open the camera
cap = cv2.VideoCapture(4)  # '0' for internal camera, '1' or higher for external cameras

if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Set the frame width and height
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Calculate the center of the frame
frame_center_x = 640 // 2
frame_center_y = 480 // 2

while not rospy.is_shutdown():
    start_time = time.time()
    
    # Capture frame from camera
    ret, frame = cap.read()

    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Upload frame to GPU if CUDA is available
    if use_cuda:
        gpu_frame = cv2.cuda_GpuMat()
        gpu_frame.upload(frame)
        frame = gpu_frame.download()  # Download frame back to CPU for further processing

    # Perform object detection using YOLOv8
    results = model(frame)

    # Create ObjectDetection message
    obj_det_msg = ObjectDetection()
    
    # Draw bounding boxes and labels on the frame and publish the results to ROS
    for result in results:
        for box in result.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = box.conf[0].tolist()
            cls = box.cls[0].tolist()
            label = model.names[int(cls)]
            
            # Draw rectangle and label
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            # Calculate the center point of the bounding box
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # Draw the center point of the bounding box
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # Display the coordinates of the center point
            cv2.putText(frame, f'({center_x}, {center_y})', (center_x, center_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            
            # Create BoundingBox message and publish
            bbox_msg = BoundingBox()
            bbox_msg.class_name = label
            bbox_msg.probability = conf
            bbox_msg.x_min = x1
            bbox_msg.y_min = y1
            bbox_msg.x_max = x2
            bbox_msg.y_max = y2
            
            # Append BoundingBox message to ObjectDetection message
            obj_det_msg.bounding_boxes.append(bbox_msg)

    # Draw the center point of the frame
    cv2.circle(frame, (frame_center_x, frame_center_y), 5, (255, 0, 0), -1)
    
    # Display the coordinates of the center point of the frame
    cv2.putText(frame, f'({frame_center_x}, {frame_center_y})', (frame_center_x + 10, frame_center_y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # Publish ObjectDetection message
    obj_det_pub.publish(obj_det_msg)

    # Display the frame
    # cv2.imshow('Camera Feed', frame)

    # Calculate and display FPS
    fps = 1.0 / (time.time() - start_time)
    print(f"FPS: {fps:.2f}")

    # Wait 1ms to see if a key is pressed, exit on 'q' key press
    if cv2.waitKey(1) == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
