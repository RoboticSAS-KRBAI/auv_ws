#!/usr/bin/env python3
import math
import rospy
from std_msgs.msg import Bool, Float32, Int16, String
from sensor_msgs.msg import Imu
from robotic_sas_auv_ros.msg import ArduinoSensor, Sensor, ObjectDetection, Heading, ObjectDifference
from nav_msgs.msg import Odometry

class Subscriber():
    def __init__(self):
        self.is_pre_calibrating = False

        self.offset_roll = 0
        self.offset_pitch = 0
        self.offset_yaw = 0
        self.offset_depth = 0  

        self.sensor = Sensor()
        self.object_detection = ObjectDetection()
        self.object_difference = ObjectDifference()

        # Publisher
        self.pub_sensor = rospy.Publisher('sensor', Sensor, queue_size=10)
        self.pub_object_difference = rospy.Publisher('object_difference', ObjectDifference, queue_size=10)
        self.pub_largest_object = rospy.Publisher('largest_object', String, queue_size=10)

        # Subscriber
        rospy.Subscriber('is_start', Bool, self.callback_is_start)
        rospy.Subscriber('/imu', Imu, self.callback_imu)
        rospy.Subscriber('/witmotion/heading', Heading, self.callback_heading)
        rospy.Subscriber('/camera/odom/sample', Odometry, self.callback_odometry)
        rospy.Subscriber('/rosserial/sensor', ArduinoSensor, self.callback_arduino_sensor)
        rospy.Subscriber('/filterYaw', Float32, self.callback_filterYaw)
        rospy.Subscriber('object_detection', ObjectDetection, self.object_detection_callback)

    def pre_calibrate(self):
        # Set offset values in order to set the initial sensor value to zero
        self.offset_roll = self.sensor.roll
        self.offset_pitch = self.sensor.pitch
        self.offset_yaw = self.sensor.yaw
        self.offset_depth = self.sensor.depth

    def get_offset(self, offset):
        # Return the given offset if the pre calibration is complete
        if not self.is_pre_calibrating:
            return offset
        else:
            return 0

    # Collect Arduino Sensor Data
    def callback_arduino_sensor(self, data: ArduinoSensor):
        self.sensor.depth = data.depth 

    # Collect Realsense Position Datas
    def callback_odometry(self, data: Odometry):
        self.sensor.pos_x = data.pose.pose.position.x
        self.sensor.pos_y = data.pose.pose.position.y
        self.sensor.pos_z = data.pose.pose.position.z

    # Collect IMU Data
    def callback_imu(self, data: Imu):
        self.sensor.roll = 180/math.pi*(math.atan2(2.0*(data.orientation.y*data.orientation.z + data.orientation.w*data.orientation.x), data.orientation.w*data.orientation.w - data.orientation.x*data.orientation.x - data.orientation.y*data.orientation.y + data.orientation.z*data.orientation.z))
        self.sensor.pitch = 180/math.pi*(math.asin(-2.0*(data.orientation.x*data.orientation.z - data.orientation.w*data.orientation.y)))
        # self.sensor.yaw = 180/math.pi*(math.atan2(2.0*(data.orientation.x*data.orientation.y + data.orientation.w*data.orientation.z), data.orientation.w*data.orientation.w + data.orientation.x*data.orientation.x - data.orientation.y*data.orientation.y - data.orientation.z*data.orientation.z))

    # Collect Heading Date
    def callback_heading(self, data: Heading):
        pass
        # self.sensor.yaw = round(data.yaw)

    def callback_filterYaw(self, data: Float32):
        self.sensor.yaw = data.data

    def object_detection_callback(self, data):
        rospy.loginfo("Received object detection message with %d bounding boxes", len(data.bounding_boxes))
    
        # Center x-coordinate of the frame
        frame_center_x = 640 // 2

        self.object_difference.object_type = "None"
        self.object_difference.x_difference = 0
        self.largest = 0
        self.x_gate = 0
        self.x_obstacle = 0
        self.x_bucket = 0
        
        for bbox in data.bounding_boxes:
            rospy.loginfo("Class: %s, Probability: %.2f, Coordinates: (%d, %d), (%d, %d)",
                        bbox.class_name, bbox.probability, bbox.x_min, bbox.y_min, bbox.x_max, bbox.y_max)
            
            # Calculate the center x-coordinate of the bounding box
            center_x = (bbox.x_min + bbox.x_max) // 2
            
            # Calculate the pixel difference between the center of the bounding box and the center of the frame
            x_difference = center_x - frame_center_x

            if bbox.class_name == "Obstacle":
                self.object_difference.object_type = bbox.class_name
                self.object_difference.x_difference = x_difference
                self.x_obstacle = bbox.x_max - bbox.x_min
            if bbox.class_name == "Gate":
                self.object_difference.object_type = bbox.class_name
                self.object_difference.x_difference = x_difference
                self.x_Gate = bbox.x_max - bbox.x_min
            if bbox.class_name == "Bucket":
                self.object_difference.object_type = bbox.class_name
                self.object_difference.x_difference = x_difference
                self.x_bucket = bbox.x_max - bbox.x_min

            self.largest = max(self.x_obstacle, self.x_gate, self.x_bucket)

            if self.largest == self.x_obstacle:
                self.pub_largest_object.publish("Obstacle")
            if self.largest == self.x_bucket:
                self.pub_largest_object.publish("Bucket")
            if self.largest == self.x_gate:
                self.pub_largest_object.publish("Gate")
                
            rospy.loginfo("Center x-coordinate of bounding box: %d", center_x)
            rospy.loginfo("Pixel difference between bounding box center and frame center: %d", x_difference)
        
        self.pub_object_difference.publish(self.object_difference)
        
    def callback_is_start(self, data: Bool):
        self.is_pre_calibrating = not data.data

        # Condition for pre calibrating
        if data.data:
            self.pub_sensor.publish(self.sensor)
        else:
            self.pre_calibrate()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_accumulator', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()