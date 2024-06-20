#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int32
from robotic_sas_auv_ros.msg import SetPoint, IsStable, Movement

class Subscriber():
    def __init__ (self):
        self.is_start = False
        self.boot_time = 0
        self.start_time = 0

        self.is_stable = IsStable()
        self.set_point = SetPoint()
        self.movement = Movement()
        self.filter = 0

        self.set_point.roll = 0 #y
        self.set_point.pitch = 0 #x
        self.set_point.yaw = -88
        self.set_point.depth = -0.4

        

        self.param_delay = rospy.get_param('/nuc/delay')
        self.param_duration = rospy.get_param('/nuc/duration')

        # Publisher
        self.pub_is_start = rospy.Publisher('is_start', Bool, queue_size=10)
        self.pub_set_point = rospy.Publisher('set_point', SetPoint, queue_size=10)
        self.pub_movement = rospy.Publisher('movement', Movement, queue_size=10)
        self.pub_constrain_pwm = rospy.Publisher('constrain_pwm', Int32, queue_size=10)
        self.pub_delay = rospy.Publisher('is_delay', Bool, queue_size=10)

        # Subscriber
        rospy.Subscriber('/rosserial/is_start', Bool, self.callback_is_start)
        rospy.Subscriber('is_stable', IsStable, self.callback_is_stable)
        rospy.Subscriber('dive',Bool,self.callback_dive)
        # rospy.Subscriber('/filterYaw',Float32,self.callback_filterYaw)

    # def callback_filterYaw(self,data:Float32):
    #     self.filter = data.data
    #     print("dsfsdfa = ",self.set_point.yaw)
    

    def set_depth(self, depth):
        # Change depth set point from the given value
        rospy.loginfo('Set Depth %s', depth)
        self.set_point.depth = depth

    def set_heading(self, heading):
        # Change yaw set point from the given value
        # pass
        rospy.loginfo('Set Sway %s', heading)
        self.set_point.yaw = heading
        # self.set_point.sway = heading

    def publish_movement(self, type, pwm):
        # Publish command and pwm value to node control
        # rospy.loginfo('Set %s %s', type, pwm)
        self.movement.type = type
        self.movement.pwm = pwm
        self.pub_movement.publish(self.movement)

    def callback_dive(self, data:Bool):
        self.dive = data

    def is_in_range(self, start_time, end_time):
        return (self.boot_time > start_time + self.param_delay and end_time is None) or (start_time + self.param_delay) < self.boot_time < (end_time + self.param_delay)

    def stop_auv(self):
        # Stop AUV
        rospy.loginfo('STOP')
        self.pub_is_start.publish(False)

    def start_auv(self):
        # Stop AUV when the timer reaches 27 secs since pre calibration
        # if self.is_in_range(30, None):
        #     self.stop_auv()
        #     return

        # # Start AUV mission
        self.pub_set_point.publish(self.set_point)
        self.pub_is_start.publish(True)
        if not self.dive.data:

            # TIMERR
            if self.is_in_range(6,16):
                # self.pub_delay.publish(False)
                self.set_heading(-88)
            if self.is_in_range(17, 22):
                # self.pub_delay.publish(True)
                self.set_heading(-168)
            if self.is_in_range(23, None):
                # self.pub_delay.publish(False)
                self.set_heading(-88)
            # if self.is_in_range(22,None):
            #     # self.pub_delay.publish(False)
            #     self.set_heading(-88)

            if self.is_in_range(6,7):
                self.pub_constrain_pwm.publish(1500)
            if self.is_in_range(7,8): 
                self.pub_constrain_pwm.publish(1490)
            if self.is_in_range(8,9):
                self.pub_constrain_pwm.publish(1480)
            if self.is_in_range(9,10):
                self.pub_constrain_pwm.publish(1470)
            if self.is_in_range(10, 11):
                self.pub_constrain_pwm.publish(1460)
            if self.is_in_range(11,12):
                self.pub_constrain_pwm.publish(1450)
            if self.is_in_range(12,13):
                self.pub_constrain_pwm.publish(1440)
            if self.is_in_range(13,14):
                self.pub_constrain_pwm.publish(1430)
            if self.is_in_range(14,15):
                self.pub_constrain_pwm.publish(1420)
            if self.is_in_range(15,16):
                self.pub_constrain_pwm.publish(1410)
            if self.is_in_range(16,None):
                self.pub_constrain_pwm.publish(1400)

    # Collect IsStable Data
    def callback_is_stable(self, data: IsStable):
        self.is_stable = data

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