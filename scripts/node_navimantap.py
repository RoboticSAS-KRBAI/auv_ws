#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu 
from std_msgs.msg import Float32
from robotic_sas_auv_ros.msg import Heading  # Ganti dengan pesan yang sesuai untuk data heading
import tf

class SensorFusion:
    def __init__(self):
        rospy.init_node('node_sensor_fusion', anonymous=True)
        
        # Subscribers untuk IMU dan heading
        self.imu_subscriber = rospy.Subscriber('/imu', Imu, self.callback_imu)
        self.heading_subscriber = rospy.Subscriber('/witmotion/heading', Heading, self.callback_heading)

        # Publisher
        self.pub_filterYaw = rospy.Publisher('/filterYaw',Float32,queue_size=10)
        
        # Inisialisasi data IMU
        self.orientation = np.zeros(4)
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)
        
        # Inisialisasi data heading
        self.yaw = 0.0
        self.mag_data = np.zeros(3)
        
        # Kalman filter variables
        self.yaw_estimate = 0.0
        self.yaw_variance = 1.0
        self.process_variance = 1e-3
        self.measurement_variance_imu = 1.0
        self.measurement_variance_compass = 0.1

        # Variabel kalibrasi
        self.yaw_calibrated = False
        self.yaw_offset = 0.0

        # Buffer untuk menyimpan history yaw
        self.yaw_history = []
        self.imu_yaw_history = []
        self.yaw_drift_threshold = 5.0

    def callback_imu(self, data: Imu):
        # Callback untuk data IMU dari topik /imu
        self.orientation = np.array([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.angular_velocity = np.array([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z])
        self.linear_acceleration = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])

        # Kalibrasi yaw IMU jika belum dikalibrasi dan ada data kompas
        if not self.yaw_calibrated and self.yaw != 0.0:
            euler = self.quaternion_to_euler(self.orientation)
            imu_yaw = np.degrees(euler[2])
            self.yaw_offset = self.yaw - imu_yaw
            self.yaw_calibrated = True

        # Dapatkan yaw dari IMU dan tambahkan offset kalibrasi
        euler = self.quaternion_to_euler(self.orientation)
        imu_yaw = np.degrees(euler[2]) + self.yaw_offset

        # Simpan yaw IMU dalam buffer history
        self.imu_yaw_history.append(imu_yaw)
        if len(self.imu_yaw_history) > 100:
            self.imu_yaw_history.pop(0)

        # Perbarui estimasi yaw menggunakan Kalman filter dengan data IMU
        self.kalman_filter_update(imu_yaw, self.measurement_variance_imu)

        # Cetak data IMU dalam derajat
        # self.print_imu_data(imu_yaw)
        self.detect_imu_drift(imu_yaw)

    def callback_heading(self, data: Heading):
        # Callback untuk data heading dari topik /witmotion/heading
        self.yaw = data.yaw
        self.mag_data = np.array([data.mag_x, data.mag_y, data.mag_z])

        # Simpan yaw kompas dalam buffer history
        self.yaw_history.append(self.yaw)
        if len(self.yaw_history) > 100:
            self.yaw_history.pop(0)

        # Perbarui estimasi yaw menggunakan Kalman filter dengan data kompas
        self.kalman_filter_update(self.yaw, self.measurement_variance_compass)

        # Cetak data heading dalam derajat
        self.print_heading_data()
        self.detect_compass_drift()

    def kalman_filter_update(self, measurement, measurement_variance):
        # Kalman filter update step
        kalman_gain = self.yaw_variance / (self.yaw_variance + measurement_variance)
        self.yaw_estimate = self.yaw_estimate + kalman_gain * (measurement - self.yaw_estimate)
        self.yaw_variance = (1 - kalman_gain) * self.yaw_variance + self.process_variance

    def detect_imu_drift(self, imu_yaw):
        if len(self.imu_yaw_history) < 2:
            return  # Tidak cukup data untuk analisis

        # Ambil nilai yaw terbaru dan yang sebelumnya
        current_yaw = self.imu_yaw_history[-1]
        previous_yaw = self.imu_yaw_history[-2]

        # Analisis perubahan pada nilai yaw
        delta_yaw = np.abs(current_yaw - previous_yaw)

        # if delta_yaw > self.yaw_drift_threshold:
        #     rospy.logwarn("IMU Yaw drift detected! Delta yaw: {:.2f}".format(delta_yaw))
        # else:
        #     rospy.loginfo("No significant IMU yaw drift. Delta yaw: {:.2f}".format(delta_yaw))

    def detect_compass_drift(self):
        if len(self.yaw_history) < 2:
            return  # Tidak cukup data untuk analisis

        # Ambil nilai yaw terbaru dan yang sebelumnya
        current_yaw = self.yaw_history[-1]
        previous_yaw = self.yaw_history[-2]

        #Analisis perubahan pada nilai yaw
        delta_yaw = np.abs(current_yaw - previous_yaw)

        # if delta_yaw > self.yaw_drift_threshold:
        #     rospy.logwarn("Compass Yaw drift detected! Delta yaw: {:.2f}".format(delta_yaw))
        # else:
        #     rospy.loginfo("No significant compass yaw drift. Delta yaw: {:.2f}".format(delta_yaw))

    # def print_imu_data(self, imu_yaw):
    #     rospy.loginfo("IMU Orientation (degrees): Yaw: {:.2f}".format(imu_yaw))
    #     rospy.loginfo("Filtered Yaw (degrees): {:.2f}".format(self.yaw_estimate))

    def print_heading_data(self):
        # rospy.loginfo("Compass Yaw (degrees): {:.2f}".format(self.yaw))
        # rospy.loginfo("Filtered Yaw (degrees): {:.2f}".format(self.yaw_estimate))
        self.pub_filterYaw.publish(self.yaw_estimate)

    def quaternion_to_euler(self, quaternion):
        # Mengonversi quaternion ke sudut Euler
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        fusion = SensorFusion()
        fusion.run()
    except rospy.ROSInterruptException:
        pass
