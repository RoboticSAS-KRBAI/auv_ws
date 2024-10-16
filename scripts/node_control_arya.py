#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Bool, Int32, String
from robotic_sas_auv_ros.msg import Error, Actuator, IsStable, ObjectDifference
import numpy as np

class DPRController: # Depth Pitch Roll
    def __init__(self, Lx, Ly):
        self.Lx = Lx
        self.Ly = Ly

        # Control matrix A
        self.A = np.array([
            [ Ly, -Ly,  Ly, -Ly],  # Roll contributions
            [ Lx,  Lx, -Lx, -Lx],  # Pitch contributions 
            [  1,   1,   1,   1]   # Depth contributions
        ])

    def control(self, control_depth, control_pitch, control_roll):
        desired_control = np.array([control_depth, control_pitch, control_roll])
        
        T = np.linalg.pinv(self.A).dot(desired_control)

        return T

class SSYController:
    def __init__(self, d):
        self.d = d
        self.sqrt2 = np.sqrt(2)

        self.M = np.array([
            [-1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2, -1/self.sqrt2],
            [1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2],
            [d/2, -d/2, -d/2, d/2],
            [1, -1, 1, -1]  # Kontribusi untuk momen yaw
        ])

    def control(self, Fx, Fy, tau):
        # Vektor gaya yang diinginkan
        F = np.array([Fx, Fy, 0, tau])  # tau dimasukkan ke koordinat keempat
        
        # Menghitung thrust yang diperlukan menggunakan pseudo-inverse matriks
        T = np.dot(np.linalg.pinv(self.M), F)

        return T
    
class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_max = 80

        self.proportional = 0
        self.integral = 0
        self.derivative = 0

        self.start_time = rospy.get_time()
        self.last_time = 0
        self.last_error = 0

    def __call__(self, error):
        dt = rospy.get_time() - (self.last_time if self.last_time is not None else self.start_time)
        d_error = error - self.last_error

        self.proportional = self.kp * error
        self.integral += -self.ki * error * dt
        self.derivative = self.kd * d_error / dt

        self.last_error = error
        self.last_time = rospy.get_time()

        return self.proportional + self.integral + self.derivative
    
class ThrusterMovement():
    def __init__(self):
        self.pwm_actuator = Actuator()
        self.pwm_actuator.thruster_1 = 1500
        self.pwm_actuator.thruster_2 = 1500
        self.pwm_actuator.thruster_3 = 1500
        self.pwm_actuator.thruster_4 = 1500
        self.pwm_actuator.thruster_5 = 1500
        self.pwm_actuator.thruster_6 = 1500
        self.pwm_actuator.thruster_7 = 1500
        self.pwm_actuator.thruster_8 = 1500
        self.pwm_actuator.thruster_9 = 1500
        self.pwm_actuator.thruster_10 = 1500

        self.pub_pwm_actuator = rospy.Publisher('pwm_actuator', Actuator, queue_size=10)

    def go_surface(self, pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4, pwm_thruster_9, pwm_thruster_10):
        self.pwm_actuator.thruster_1 = pwm_thruster_1
        self.pwm_actuator.thruster_2 = pwm_thruster_2
        self.pwm_actuator.thruster_3 = pwm_thruster_3
        self.pwm_actuator.thruster_4 = pwm_thruster_4
        self.pwm_actuator.thruster_9 = pwm_thruster_9
        self.pwm_actuator.thruster_10 = pwm_thruster_10

    def surge_sway_yaw(self, pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4, pwm_thruster_9, pwm_thruster_10):
        self.pwm_actuator.thruster_1 = pwm_thruster_1
        self.pwm_actuator.thruster_2 = pwm_thruster_2
        self.pwm_actuator.thruster_3 = pwm_thruster_3
        self.pwm_actuator.thruster_4 = pwm_thruster_4
        self.pwm_actuator.thruster_9 = pwm_thruster_9
        self.pwm_actuator.thruster_10 = pwm_thruster_10

    def depth_pitch_roll(self, pwm_thruster_5, pwm_thruster_6, pwm_thruster_7, pwm_thruster_8):
        self.pwm_actuator.thruster_5 = pwm_thruster_5
        self.pwm_actuator.thruster_6 = pwm_thruster_6
        self.pwm_actuator.thruster_7 = pwm_thruster_7
        self.pwm_actuator.thruster_8 = pwm_thruster_8

    def stop(self):
        self.pwm_actuator.thruster_1 = 1500
        self.pwm_actuator.thruster_2 = 1500
        self.pwm_actuator.thruster_3 = 1500
        self.pwm_actuator.thruster_4 = 1500
        self.pwm_actuator.thruster_5 = 1500
        self.pwm_actuator.thruster_6 = 1500
        self.pwm_actuator.thruster_7 = 1500
        self.pwm_actuator.thruster_8 = 1500
        self.pwm_actuator.thruster_9 = 1500
        self.pwm_actuator.thruster_10 = 1500

    def publish(self):
        print('Publish PWM')
        print(self.pwm_actuator)
        self.pub_pwm_actuator.publish(self.pwm_actuator)

class Subscriber():
    def __init__(self):
        self.is_start = False
        self.move = "stop"
        self.error = Error()
        self.is_stable = IsStable()
        self.ssyController = SSYController(1)
        self.dprController = DPRController(0.5, 0.5)
        self.movement = ThrusterMovement()
        self.movement.stop()

        self.param_delay = rospy.get_param('/nuc/delay')
        self.param_arming_duration = rospy.get_param('/nuc/arming_duration')

        self.pid_depth = PID(1700, 0, 200)
        self.pid_roll = PID(15, 0, 0)  #PID(15, 0, 0) bagus
        self.pid_pitch = PID(20, 0, 7)  #PID(15, 0, 7) bagus
        # self.pid_sway = PID(6, 0, 0)
        
        # self.pid_yaw = PID(17, 0, 10)
        self.pid_yaw = PID(15, 0, 1)  #PID(10, 0, 1) bagus
        self.is_pre_calibrating = False
        self.dive = False

        self.thrust_depth_pitch_roll = [0,0,0,0]
        self.offset_depth_pitch_roll = [0,0,0,0]
        self.thrust_surge_sway_yaw = [0,0,0,0]
        self.offset_surge_sway_yaw = [0,0,0,0]

        self.constrain_pwm_min = 0
        self.constrain_pwm_max = 0

        self.is_stable.depth = False
        self.is_stable.yaw = False
        self.is_stable.pitch = False
        self.is_stable.roll = False

        # Subscriber
        rospy.Subscriber('constrain_pwm', Int32, self.callback_constrain_pwm)
        rospy.Subscriber('error', Error, self.callback_error)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)
        rospy.Subscriber('move', String, self.callback_move)
        # rospy.Subscriber('object_difference', ObjectDifference, self.callback_object_difference)

        self.pub_dive = rospy.Publisher('dive',Bool,queue_size=10)

    def constrain(self, value, _min, _max):
        return min(max(value, _min), _max)
    
    def pre_calibrate(self):
        self.offset_surge_sway_yaw[0] = self.thrust_surge_sway_yaw[0]
        self.offset_surge_sway_yaw[1] = self.thrust_surge_sway_yaw[1]
        self.offset_surge_sway_yaw[2] = self.thrust_surge_sway_yaw[2]
        self.offset_surge_sway_yaw[3] = self.thrust_surge_sway_yaw[3]

        self.offset_depth_pitch_roll[0] = self.thrust_depth_pitch_roll[0]
        self.offset_depth_pitch_roll[1] = self.thrust_depth_pitch_roll[1]
        self.offset_depth_pitch_roll[2] = self.thrust_depth_pitch_roll[2]
        self.offset_depth_pitch_roll[3] = self.thrust_depth_pitch_roll[3]

    def get_offset(self, offset):
        return offset if not self.is_pre_calibrating else 0
    
    def surface(self):
        pwm_thruster_1 = 1500
        pwm_thruster_2 = 1500
        pwm_thruster_3 = 1500
        pwm_thruster_4 = 1500
        pwm_thruster_9 = 1500
        pwm_thruster_10 = 1500
        self.movement.go_surface(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4, pwm_thruster_9, pwm_thruster_10)

    def surge_yaw(self):
        min_pwm = 1000
        max_pwm = 2000
    
        pwm_thruster_1 = self.constrain(1500 - (self.thrust_surge_sway_yaw[1] * 500 - self.get_offset(self.offset_surge_sway_yaw[1])), min_pwm, max_pwm)
        pwm_thruster_2 = self.constrain(1500 - (self.thrust_surge_sway_yaw[0] * 500 - self.get_offset(self.offset_surge_sway_yaw[0])), min_pwm, max_pwm)
        pwm_thruster_3 = self.constrain(1500 - (self.thrust_surge_sway_yaw[3] * 500 - self.get_offset(self.offset_surge_sway_yaw[3])), min_pwm, max_pwm)
        pwm_thruster_4 = self.constrain(1500 - (self.thrust_surge_sway_yaw[2] * 500 - self.get_offset(self.offset_surge_sway_yaw[2])), min_pwm, max_pwm)
        pwm_thruster_9 = self.constrain(1500 - (self.thrust_surge_sway_yaw[1] * 500 - self.get_offset(self.offset_surge_sway_yaw[1])), self.constrain_pwm_min, self.constrain_pwm_max)
        pwm_thruster_10 = self.constrain(1500 - (self.thrust_surge_sway_yaw[0] * 500 - self.get_offset(self.offset_surge_sway_yaw[0])), self.constrain_pwm_min, self.constrain_pwm_max)
        
        self.movement.surge_sway_yaw(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4, pwm_thruster_9, pwm_thruster_10)

    def sway_yaw(self):
        min_pwm = 1000
        max_pwm = 2000
    
        pwm_thruster_1 = self.constrain(1500 - (self.thrust_surge_sway_yaw[1] * 500 - self.get_offset(self.offset_surge_sway_yaw[1])), min_pwm, max_pwm)
        pwm_thruster_2 = self.constrain(1500 - (self.thrust_surge_sway_yaw[0] * 500 - self.get_offset(self.offset_surge_sway_yaw[0])), min_pwm, max_pwm)
        pwm_thruster_3 = self.constrain(1500 - (self.thrust_surge_sway_yaw[3] * 500 - self.get_offset(self.offset_surge_sway_yaw[3])), min_pwm, max_pwm)
        pwm_thruster_4 = self.constrain(1500 - (self.thrust_surge_sway_yaw[2] * 500 - self.get_offset(self.offset_surge_sway_yaw[2])), min_pwm, max_pwm)
        pwm_thruster_9 = 1500
        #self.constrain(1500 - (self.thrust_surge_sway_yaw[0] * 500 - self.get_offset(self.offset_surge_sway_yaw[0])), self.constrain_pwm_min, self.constrain_pwm_max)
        pwm_thruster_10 = 1500
        #self.constrain(1500 - (self.thrust_surge_sway_yaw[1] * 500 - self.get_offset(self.offset_surge_sway_yaw[1])), self.constrain_pwm_min, self.constrain_pwm_max)
        
        self.movement.surge_sway_yaw(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4, pwm_thruster_9, pwm_thruster_10)

    def depth_pitch_roll(self):
        min_pwm = 1000
        max_pwm = 2000
        pwm_thruster_5 = self.constrain(1500 + (self.thrust_depth_pitch_roll[2] - self.get_offset(self.offset_depth_pitch_roll[0])), min_pwm, max_pwm)
        pwm_thruster_6 = self.constrain(1500 - (self.thrust_depth_pitch_roll[1] - self.get_offset(self.offset_depth_pitch_roll[1])), min_pwm, max_pwm)
        pwm_thruster_7 = self.constrain(1500 + (self.thrust_depth_pitch_roll[0] - self.get_offset(self.offset_depth_pitch_roll[2])), min_pwm, max_pwm)
        pwm_thruster_8 = self.constrain(1500 - (self.thrust_depth_pitch_roll[3] - self.get_offset(self.offset_depth_pitch_roll[3])), min_pwm, max_pwm)
        self.movement.depth_pitch_roll(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    def stabilize_depth_pitch_roll(self, error_depth, error_pitch, error_roll):
        self.thrust_depth_pitch_roll = self.dprController.control(self.pid_depth(error_depth), self.pid_pitch(error_pitch), -(self.pid_roll(error_roll)))

    def stabilize_surge_yaw(self, error):
        self.t_yaw = np.interp(self.pid_yaw(error), [-500, 500], [-3, 3])
        self.thrust_surge_sway_yaw = self.ssyController.control(0, 2, self.t_yaw)
    
    def stabilize_surge(self, error):
        self.t_yaw = np.interp(self.pid_yaw(error), [-500, 500], [-3, 3])
        self.thrust_surge_sway_yaw = self.ssyController.control(0, 2, 0)

    def stabilize_sway_yaw_left(self, error):
        self.t_yaw = np.interp(self.pid_yaw(error), [-500, 500], [-3, 3])
        self.thrust_surge_sway_yaw = self.ssyController.control(4, 1, self.t_yaw)

    def stabilize_sway_yaw_right(self, error):
        self.t_yaw = np.interp(self.pid_yaw(error), [-500, 500], [-3, 3])
        self.thrust_surge_sway_yaw = self.ssyController.control(-4, 1, self.t_yaw)

    def stabilize_yaw(self, error):
        self.t_yaw = np.interp(self.pid_yaw(error), [-500, 500], [-3, 3])
        self.thrust_surge_sway_yaw = self.ssyController.control(0, 0, self.t_yaw)

    def stabilize_yaw_right(self, error):
        self.t_yaw = np.interp(self.pid_yaw(error), [-500, 500], [-3, 3])
        self.thrust_surge_sway_yaw = self.ssyController.control(0, 0, 0.75)

    def stabilize_yaw_left(self, error):
        self.t_yaw = np.interp(self.pid_yaw(error), [-500, 500], [-3, 3])
        self.thrust_surge_sway_yaw = self.ssyController.control(0, 0, -0.7)

    def callback_move(self, data: String):
        self.move = data.data

    def callback_constrain_pwm(self, data: Int32):
        self.constrain_pwm_min = data.data
        self.constrain_pwm_max = (1500 - data.data)+1500
        
    def callback_error(self, data: Error):
        self.stabilize_depth_pitch_roll(data.depth, data.pitch, data.roll)
        if self.move == "forward":
            self.stabilize_surge(data.yaw)
        elif self.move == "forward_yaw":
            self.stabilize_surge_yaw(data.yaw)
        elif self.move == "sway_left":
            self.stabilize_sway_yaw_left(data.yaw)
        elif self.move == "sway_right":
            self.stabilize_sway_yaw_right(data.yaw)
        elif self.move == "yaw":
            self.stabilize_yaw(data.yaw)
        elif self.move == "yaw_right":
            self.stabilize_yaw_right(data.yaw)
        elif self.move == "yaw_left":
            self.stabilize_yaw_left(data.yaw)
        

    def stabilize(self):
        if self.move == "forward_yaw" or self.move == "camera" or self.move == "forward":
            self.surge_yaw()
            # self.depth_pitch_roll()
        elif self.move == "sway_left" or self.move == "sway_right" or self.move == "yaw" or self.move == "yaw_right" or self.move == "yaw_left":
            self.sway_yaw()
            # self.depth_pitch_roll()
        self.depth_pitch_roll()
        
        if self.move == "surface":
            self.surface()
            
    def callback_is_start(self, data: Bool):
        self.is_pre_calibrating = not data.data

        # Wait for a secs to await sensor value changes / spikes after pre calibrating
        if not self.is_pre_calibrating and self.start_time + self.param_arming_duration > rospy.get_time():
            rospy.loginfo('READY TO DIVE...')
            self.pub_dive.publish(True)
            return
        self.pub_dive.publish(False)

        # Condition for pre calibrating
        if data.data:
            self.stabilize()
        else:
            self.start_time = rospy.get_time()
            self.pre_calibrate()
            self.movement.stop()

        self.movement.publish()
    
    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_control', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()