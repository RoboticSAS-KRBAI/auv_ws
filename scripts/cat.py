#!/usr/bin/env python3
import numpy as np
from robotic_sas_auv_ros.msg import Actuator
import rospy
class ROVController:
    def __init__(self, d):
        self.d = d
        self.sqrt2 = np.sqrt(2)
        
        # Matriks transformasi
        self.M = np.array([
            [1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2, 1/self.sqrt2],
            [1/self.sqrt2, -1/self.sqrt2, -1/self.sqrt2, 1/self.sqrt2],
            [d/2, -d/2, -d/2, d/2]
        ])

    def control(self, Fx, Fy, tau):
        # Vektor gaya yang diinginkan
        F = np.array([Fx, Fy, tau])
        
        # Menghitung thrust yang diperlukan menggunakan pseudo-inverse matriks
        T = np.linalg.pinv(self.M).dot(F)

        return T

    def spin(self) :
        rospy.spin()


if __name__ == '__main__' :

    rospy.init_node("gerak", anonymous=False)

    thrust = Actuator()

    publish_thrush = rospy.Publisher('pwm_actuator', Actuator, queue_size=10)



    # Parameter ROV
    d = 1.0  # Jarak dari pusat ROV ke thruster

    # Buat controller
    controller = ROVController(d)

    # Gaya yang diinginkan (Fx, Fy, tau)
    Fx = 1# Gaya maju
    Fy = 0  # Gaya ke samping
    tau = 0 # Momen yaw

    # Hitung thrust yang diperlukan untuk setiap thruster
    thrusts = controller.control(Fx, Fy, tau)
    thrust.thruster_1 = thrusts[0]
    thrust.thruster_2 = thrusts[1]
    thrust.thruster_3 = thrusts[2]
    thrust.thruster_4 = thrusts[3]
    # publish_thrush.publish(thrust)

    # Tampilkan thrust yang diperlukan untuk setiap thruster
    print(f'Thrusts: {thrusts}')
    print(1500 - thrusts[0]*500)
    print(1500 - thrusts[1]*500)
    print(1500 - thrusts[2]*500)
    print(1500 - thrusts[3]*500)

    controller.spin()
    