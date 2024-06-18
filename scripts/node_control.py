#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Bool, Int32
from robotic_sas_auv_ros.msg import Error, Actuator, Movement, IsStable
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
        
        # Transformation matrix
        self.M = np.array([
            [1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2],
            [1/self.sqrt2, -1/self.sqrt2, -1/self.sqrt2, 1/self.sqrt2],
            [d/2, -d/2, -d/2, d/2]
        ])

    def control(self, Fx, Fy, tau):
        # Desired force vector
        F = np.array([Fx, Fy, tau])
        
        # Calculate required thrust using the pseudo-inverse of the matrix
        T = np.linalg.pinv(self.M).dot(F)

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

        # if self.integral > self.integral_max:
        #     self.integral = self.integral_max
        # elif self.integral < -self.integral_max:
        #     self.integral = -self.integral_max

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

    def boost(self, pwm_thruster_9, pwm_thruster_10):
        self.pwm_actuator.thruster_9 = pwm_thruster_9
        self.pwm_actuator.thruster_10 = pwm_thruster_10

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
        print('Publish PWM', self.pwm_actuator)
        self.pub_pwm_actuator.publish(self.pwm_actuator)

class Subscriber():
    def __init__(self):
        self.error = Error()
        self.is_stable = IsStable()
        self.ssyController = SSYController(1)
        self.dprController = DPRController(0.5, 0.5)
        self.movement = ThrusterMovement()
        self.movement.stop()
        
        self.param_arming_duration = rospy.get_param('/nuc/arming_duration')

        self.pid_depth = PID(1600, 0, 200)
        self.pid_roll = PID(600, 0, 10)
        # self.pid_pitch = PID(1200, 0, 200) 
        self.pid_pitch = PID(2000, 0, 1000) # P 2000 I 0 D 500 jika ingin pakai thrust 9 & 10
        
        self.pid_yaw = PID(6, 0, 0)

        self.angka = 0

        self.start_time = 0
        self.is_armed = False
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

        # self.offset_roll = 0
        # self.offset_pitch = 0
        # self.offset_yaw = 0
        
        self.offset_surge = 0
        # self.offset_sway = 0
        # self.offset_depth = 0

        # self.pwm_roll = 0
        # self.pwm_pitch = 0
        # self.pwm_yaw = 0

        self.pwm_surge = 0
        # self.pwm_sway = 0
        # self.pwm_depth = 0

        # Subscriber
        rospy.Subscriber('constrain_pwm', Int32, self.callback_constrain_pwm)
        rospy.Subscriber('error', Error, self.callback_error)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)
        rospy.Subscriber('movement', Movement, self.callback_movement)
        rospy.Subscriber('is_stable', IsStable, self.callback_is_stable)

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

        # self.offset_roll = self.pwm_roll
        # self.offset_pitch = self.pwm_pitch
        # self.offset_yaw = self.pwm_yaw
        
        self.offset_surge = self.pwm_surge
        # self.offset_sway = self.pwm_sway
        # self.offset_depth = self.pwm_depth

    def get_offset(self, offset):
        return offset if not self.is_pre_calibrating else 0

    def surge_sway_yaw(self):
        min_pwm = 1000
        max_pwm = 2000
        pwm_thruster_1 = self.constrain(1500 - (self.thrust_surge_sway_yaw[0] * 500 - self.get_offset(self.offset_surge_sway_yaw[0])), min_pwm, max_pwm)
        pwm_thruster_2 = self.constrain(1500 - (self.thrust_surge_sway_yaw[1] * 500 - self.get_offset(self.offset_surge_sway_yaw[1])), min_pwm, max_pwm)
        pwm_thruster_3 = self.constrain(1500 - (self.thrust_surge_sway_yaw[2] * 500 - self.get_offset(self.offset_surge_sway_yaw[2])), min_pwm, max_pwm)
        pwm_thruster_4 = self.constrain(1500 - (self.thrust_surge_sway_yaw[3] * 500 - self.get_offset(self.offset_surge_sway_yaw[3])), min_pwm, max_pwm)
        pwm_thruster_9 = self.constrain(1500 - (self.thrust_surge_sway_yaw[0] * 500 - self.get_offset(self.offset_surge_sway_yaw[0])), self.constrain_pwm_min, self.constrain_pwm_max)
        #self.pwm_surge
        
        pwm_thruster_10 = self.constrain(1500 - (self.thrust_surge_sway_yaw[1] * 500 - self.get_offset(self.offset_surge_sway_yaw[1])), self.constrain_pwm_min, self.constrain_pwm_max)
        #self.pwm_surge
        
        self.movement.surge_sway_yaw(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4, pwm_thruster_9, pwm_thruster_10)

    def depth_pitch_roll(self):
        min_pwm = 1000
        max_pwm = 2000
        pwm_thruster_5 = self.constrain(1500 + (self.thrust_depth_pitch_roll[0] - self.get_offset(self.offset_depth_pitch_roll[0])), min_pwm, max_pwm)
        pwm_thruster_6 = self.constrain(1500 - (self.thrust_depth_pitch_roll[1] - self.get_offset(self.offset_depth_pitch_roll[1])), min_pwm, max_pwm)
        pwm_thruster_7 = self.constrain(1500 + (self.thrust_depth_pitch_roll[2] - self.get_offset(self.offset_depth_pitch_roll[2])), min_pwm, max_pwm)
        pwm_thruster_8 = self.constrain(1500 - (self.thrust_depth_pitch_roll[3] - self.get_offset(self.offset_depth_pitch_roll[3])), min_pwm, max_pwm)
        self.movement.depth_pitch_roll(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)


    # def surge_yaw(self):
    #     min_pwm = 1000
    #     max_pwm = 2000
    #     pwm_thruster_1 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)) + (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #     pwm_thruster_2 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)) - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #     pwm_thruster_3 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)) + (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #     pwm_thruster_4 = self.constrain(1500 - (self.pwm_surge - self.get_offset(self.offset_surge)) - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #     self.movement.surge_sway_yaw(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4)

    # def surge(self):
    #     min_pwm = 1000
    #     max_pwm = 2000
    #     pwm_thruster_1 = self.constrain(1500 - (self.thrust_surge_sway_yaw[0] * 500 - self.get_offset(self.offset_surge_sway_yaw[0])), min_pwm, max_pwm)
    #     pwm_thruster_2 = self.constrain(1500 - (self.thrust_surge_sway_yaw[1] * 500 - self.get_offset(self.offset_surge_sway_yaw[1])), min_pwm, max_pwm)
    #     pwm_thruster_3 = self.constrain(1500 - (self.thrust_surge_sway_yaw[2] * 500 - self.get_offset(self.offset_surge_sway_yaw[2])), min_pwm, max_pwm)
    #     pwm_thruster_4 = self.constrain(1500 - (self.thrust_surge_sway_yaw[3] * 500 - self.get_offset(self.offset_surge_sway_yaw[3])), min_pwm, max_pwm)
    #     self.movement.surge_sway_yaw(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4)

    # def sway(self):
    #     min_pwm = 1200
    #     max_pwm = 1800
    #     pwm_thruster_1 = self.constrain(1500 + (self.pwm_sway - self.get_offset(self.offset_sway)), min_pwm, max_pwm)
    #     pwm_thruster_2 = self.constrain(1500 - (self.pwm_sway - self.get_offset(self.offset_sway)), min_pwm, max_pwm)
    #     pwm_thruster_3 = self.constrain(1500 - (self.pwm_sway - self.get_offset(self.offset_sway)), min_pwm, max_pwm)
    #     pwm_thruster_4 = self.constrain(1500 + (self.pwm_sway - self.get_offset(self.offset_sway)), min_pwm, max_pwm)
    #     self.movement.surge_sway_yaw(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4)

    # def yaw(self):
    #     min_pwm = 1200
    #     max_pwm = 1800
    #     pwm_thruster_1 = self.constrain(1500 + (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #     pwm_thruster_2 = self.constrain(1500 - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #     pwm_thruster_3 = self.constrain(1500 + (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #     pwm_thruster_4 = self.constrain(1500 - (self.pwm_yaw - self.get_offset(self.offset_yaw)), min_pwm, max_pwm)
    #     self.movement.surge_sway_yaw(pwm_thruster_1, pwm_thruster_2, pwm_thruster_3, pwm_thruster_4)

    # def depth_roll_pitch(self):
    #     min_pwm = 1200
    #     max_pwm = 1800
    #     pwm_thruster_5 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)) - (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_6 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)) + (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_7 = self.constrain(1500 - (self.pwm_depth - self.get_offset(self.offset_depth)) + (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_8 = self.constrain(1500 - (self.pwm_depth - self.get_offset(self.offset_depth)) - (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)  
    #     self.movement.depth_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    # def depth_roll(self):
    #     min_pwm = 1200
    #     max_pwm = 1800
    #     pwm_thruster_5 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)) + (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
    #     pwm_thruster_6 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)) - (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
    #     pwm_thruster_7 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)) + (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
    #     pwm_thruster_8 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)) - (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)  
    #     self.movement.depth_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    # def depth_pitch(self):
    #     min_pwm = 1000
    #     max_pwm = 2000
    #     pwm_thruster_5 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_6 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_7 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)) - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_8 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)) - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     self.movement.depth_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    # def roll_pitch(self):
    #     min_pwm = 1200
    #     max_pwm = 1800
    #     pwm_thruster_5 = self.constrain(1500 + (self.pwm_roll - self.get_offset(self.offset_roll)) - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_6 = self.constrain(1500 - (self.pwm_roll - self.get_offset(self.offset_roll)) - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_7 = self.constrain(1500 + (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_8 = self.constrain(1500 - (self.pwm_roll - self.get_offset(self.offset_roll)) + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)  
    #     self.movement.depth_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)
    
    # def depth(self):
    #     min_pwm = 1200
    #     max_pwm = 1800
    #     pwm_thruster_5 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)), min_pwm, max_pwm)
    #     pwm_thruster_6 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)), min_pwm, max_pwm)
    #     pwm_thruster_7 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)), min_pwm, max_pwm)
    #     pwm_thruster_8 = self.constrain(1500 + (self.pwm_depth - self.get_offset(self.offset_depth)), min_pwm, max_pwm)  
    #     self.movement.depth_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)
    
    # def roll(self):
    #     min_pwm = 1200
    #     max_pwm = 1800
    #     pwm_thruster_5 = self.constrain(1500 + (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
    #     pwm_thruster_6 = self.constrain(1500 - (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
    #     pwm_thruster_7 = self.constrain(1500 + (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)
    #     pwm_thruster_8 = self.constrain(1500 - (self.pwm_roll - self.get_offset(self.offset_roll)), min_pwm, max_pwm)  
    #     self.movement.depth_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    # def pitch(self):
    #     min_pwm = 1200
    #     max_pwm = 1800
    #     pwm_thruster_5 = self.constrain(1500 - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_6 = self.constrain(1500 - (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_7 = self.constrain(1500 + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)
    #     pwm_thruster_8 = self.constrain(1500 + (self.pwm_pitch - self.get_offset(self.offset_pitch)), min_pwm, max_pwm)  
    #     self.movement.depth_roll_pitch(pwm_thruster_5, pwm_thruster_6, pwm_thruster_7,pwm_thruster_8)

    def stabilize_depth_pitch_roll(self, error_depth, error_pitch, error_roll):
        self.thrust_depth_pitch_roll = self.dprController.control(self.pid_depth(error_depth), self.pid_pitch(error_pitch), self.pid_roll(error_roll))
   
    def stabilize_surge_sway_yaw(self, error):
        self.t_surge_sway_yaw = np.interp(self.pid_yaw(error), [-500, 500], [-6, 6])
        self.thrust_surge_sway_yaw = self.ssyController.control(2, self.t_surge_sway_yaw, 0)

    #Check if Stable
    def callback_is_stable(self, data: IsStable):
        self.is_stable.depth = data.depth
        if self.is_stable.depth:
            self.angka = 1

        self.is_stable.yaw = data.yaw
        self.is_stable.pitch = data.pitch
        self.is_stable.roll = data.roll

    # Collect Constrain PWM
    def callback_constrain_pwm(self, data: Int32):
        self.constrain_pwm_min = data.data
        self.constrain_pwm_max = (1500-data.data)+1500
        
    # Collect Error Data
    def callback_error(self, data: Error):
        self.stabilize_depth_pitch_roll(data.depth, data.pitch, data.roll)
        self.stabilize_surge_sway_yaw(data.yaw)

    # Collect Movement Data
    def callback_movement(self, data: Movement):
        if data.type == 'SURGE':
            self.pwm_surge = data.pwm
        if data.type == 'SWAY':
            self.pwm_sway = data.pwm
        if data.type == 'YAW':
            self.pwm_yaw = data.pwm
        if data.type == 'depth':
            self.pwm_depth = data.pwm
        if data.type == 'ROLL':
            self.pwm_roll = data.pwm
        if data.type == 'PITCH':
            self.pwm_pitch = data.pwm
        if data.type == 'STOP':
            self.movement.stop()

    def stabilize(self):
        self.depth_pitch_roll()
        # if self.is_stable.depth:
        self.surge_sway_yaw()
        
        
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