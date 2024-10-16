#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int32, Bool, Int8
from robotic_sas_auv_ros.msg import SetPoint, IsStable, Movement, ObjectDifference

class Subscriber():
    def __init__ (self):
        self.is_start = False
        self.boot_time = 0
        self.start_time = 0
        self.bucket = False
        self.bucket_detected = False
        self.flag = 0
        self.yaw = -90
        self.move = "stop"

        self.start = 0
        self.current_time = 0
        self.elapsed_time = 0
        self.delay = 3
        self.start_delay = False

        self.is_stable = IsStable()
        self.set_point = SetPoint()
        self.movement = Movement()
        self.object_difference = ObjectDifference()
        self.filter = 0

        self.set_point.roll = 0 #y
        self.set_point.pitch = 0 #x
        self.set_point.yaw = self.yaw
        self.set_point.depth = -0.45
        self.towards_bucket = False

        self.param_delay = rospy.get_param('/nuc/delay')
        self.param_duration = rospy.get_param('/nuc/duration')

        # Publisher
        self.pub_is_start = rospy.Publisher('is_start', Bool, queue_size=10)
        self.pub_set_point = rospy.Publisher('set_point', SetPoint, queue_size=10)
        self.pub_movement = rospy.Publisher('movement', Movement, queue_size=10)
        self.pub_constrain_pwm = rospy.Publisher('constrain_pwm', Int32, queue_size=10)
        self.pub_move = rospy.Publisher('move', String, queue_size=10)

        # Subscriber
        rospy.Subscriber('/rosserial/is_start', Bool, self.callback_is_start)
        rospy.Subscriber('dive', Bool, self.callback_dive)
        rospy.Subscriber('/rosserial/bucket_detected', Bool, self.callback_bucket)
        rospy.Subscriber('flag', Int8, self.callback_flag)
        rospy.Subscriber('object_difference', ObjectDifference, self.callback_object_difference)

    def set_heading(self, heading):
        # Change yaw set point from the given value
        rospy.loginfo('Set Yaw %s', heading)
        self.set_point.yaw = heading

    def delay_time(self, event):
        self.current_time = rospy.get_time()
        self.elapsed_time = self.current_time - self.start

    def callback_object_difference(self, data: ObjectDifference):
        self.object_difference.object_type = data.object_type
        self.object_difference.x_difference = data.x_difference

    def callback_dive(self, data: Bool):
        self.dive = data

    def callback_flag(self, data: Int8):
        self.flag = data.data
    
    def callback_bucket(self, data: Bool):
        self.bucket = data.data
        if self.bucket:
            self.bucket_detected = True

    def is_in_range(self, start_time, end_time):
        return (self.boot_time > start_time + self.param_delay and end_time is None) or (start_time + self.param_delay) < self.boot_time < (end_time + self.param_delay)

    def stop_auv(self):
        # Stop AUV
        rospy.loginfo('STOP')
        self.pub_is_start.publish(False)

    def start_auv(self):
        # Stop AUV when the timer reaches 27 secs since pre calibration
        if self.is_in_range(23, None):
            self.stop_auv()
            return
        
        # Start AUV mission
        self.pub_set_point.publish(self.set_point)
        self.pub_is_start.publish(True)

        if not self.dive.data:
            if self.is_in_range(6, 9): 
                rospy.loginfo("Forward")
                self.pub_move.publish("forward")
                self.set_heading(self.yaw)
            if self.is_in_range(10, 22): 
                rospy.loginfo("Forward_yaw")
                self.pub_move.publish("forward_yaw")
                self.set_heading(self.yaw)
            # if self.is_in_range(23, 28): 
            #     rospy.loginfo("Sway_Right")
            #     self.pub_move.publish("sway_right")
            # if self.is_in_range(29, 32): 
            #     rospy.loginfo("Sway_Left")
            #     self.pub_move.publish("sway_left")
            # if self.is_in_range(29, 33): 
            #     rospy.loginfo("Yaw_Right")
            #     self.pub_move.publish("yaw_right")
            # if self.is_in_range(34, 39): 
            #     rospy.loginfo("Yaw_Left")
            #     self.pub_move.publish("yaw_left")
            # if self.is_in_range(40, 45): 
            #     rospy.loginfo("YAW")
            #     self.pub_move.publish("yaw")
            #     self.set_heading(90)
            # if self.is_in_range(46, None): 
            #     rospy.loginfo("SURFACE")
            #     self.pub_move.publish("surface")
            #     self.set_point.depth = -0.8
            if self.is_in_range(6,7):
                self.pub_constrain_pwm.publish(1500)
            if self.is_in_range(7,8): 
                self.pub_constrain_pwm.publish(1400)
            if self.is_in_range(8,9):
                self.pub_constrain_pwm.publish(1200)
            if self.is_in_range(9,None):
                self.pub_constrain_pwm.publish(1100)
            # if self.is_in_range(10, None):
            #     self.pub_constrain_pwm.publish(1100)
            # if self.is_in_range(11,12):
            #     self.pub_constrain_pwm.publish(1250)
            # if self.is_in_range(12,13):
            #     self.pub_constrain_pwm.publish(1200)
            # if self.is_in_range(13,14):
            #     self.pub_constrain_pwm.publish(1150)
            # if self.is_in_range(14,None):
            #     self.pub_constrain_pwm.publish(1100)
            # if self.is_in_range(15,16):
            #     self.pub_constrain_pwm.publish(1410)
            # if self.is_in_range(16,None):
            #     self.pub_constrain_pwm.publish(1400)

    def callback_is_start(self, data: Bool):
        if data.data:
            # Set start time
            if not self.is_start:
                self.start_time = rospy.get_time()
                self.is_start = True

            # Generate boot time
            self.boot_time = rospy.get_time() - self.start_time
            
            # Wait for a secs to tell other nodes (accumulator & control) to calibrate
            if self.boot_time < self.param_delay:
                rospy.loginfo('STARTING...')
                self.pub_is_start.publish(False)
                return

            # Timer condition
            if self.boot_time < self.param_duration if self.param_duration >= 0 else True:
                self.start_auv()
            else:
                self.stop_auv()

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_guidance', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass