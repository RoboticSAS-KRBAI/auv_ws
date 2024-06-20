#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from robotic_sas_auv_ros.msg import Heading  # Ganti dengan pesan yang sesuai untuk data heading
import tf

class SensorDriftDetector:
    def __init__(self):
        rospy.init_node('sensor_drift_detector', anonymous=True)
        
        # Subscribers untuk IMU dan heading
        self.imu_subscriber = rospy.Subscriber('/imu', Imu, self.callback_imu)
        self.heading_subscriber = rospy.Subscriber('/witmotion/heading', Heading, self.callback_heading)
        
        # Inisialisasi data IMU
        self.orientation = np.zeros(4)
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)
        
        # Inisialisasi data heading
        self.yaw = 0.0
        self.mag_data = np.zeros(3)
        
        # Inisialisasi covariance matrices
        self.orientation_covariance = np.zeros((3, 3))
        self.angular_velocity_covariance = np.zeros((3, 3))
        self.linear_acceleration_covariance = np.zeros((3, 3))
        
        # Buffers untuk menyimpan history
        self.angular_velocity_cov_history = []
        self.yaw_history = []

        # Variabel kalibrasi
        self.yaw_calibrated = False
        self.yaw_offset = 0.0

    def callback_imu(self, data:Imu):
        # Callback untuk data IMU dari topik /imu
        self.orientation = np.array([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.angular_velocity = np.array([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z])
        self.linear_acceleration = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])
        
        # Covariance matrices
        self.orientation_covariance = np.array(data.orientation_covariance).reshape((3, 3))
        self.angular_velocity_covariance = np.array(data.angular_velocity_covariance).reshape((3, 3))
        self.linear_acceleration_covariance = np.array(data.linear_acceleration_covariance).reshape((3, 3))

        # Kalibrasi yaw IMU jika belum dikalibrasi
        if not self.yaw_calibrated and self.yaw_history:
            euler = self.quaternion_to_euler(self.orientation)
            imu_yaw = np.degrees(euler[2])
            self.yaw_offset = self.yaw_history[-1] - imu_yaw
            self.yaw_calibrated = True

        # Simpan covariance matrix angular velocity dalam buffer history
        self.angular_velocity_cov_history.append(self.angular_velocity_covariance)
        
        # Jaga ukuran buffer agar tidak terlalu besar
        if len(self.angular_velocity_cov_history) > 100:
            self.angular_velocity_cov_history.pop(0)
        
        # Analisis drifting pada arah yaw (angular velocity z)
        self.detect_imu_drift()

        # Cetak data IMU dalam derajat
        self.print_imu_data()

    def callback_heading(self, data: Heading):
        # Callback untuk data heading dari topik /witmotion/heading
        self.yaw = data.yaw
        self.mag_data = np.array([data.mag_x, data.mag_y, data.mag_z])

        # Simpan nilai yaw dalam buffer history
        self.yaw_history.append(self.yaw)
        
        # Jaga ukuran buffer agar tidak terlalu besar
        if len(self.yaw_history) > 100:
            self.yaw_history.pop(0)
        
        # Analisis drifting pada arah yaw
        self.detect_heading_drift()

        # Cetak data heading dalam derajat
        self.print_heading_data()

    def detect_imu_drift(self):
        if len(self.angular_velocity_cov_history) < 2:
            return  # Tidak cukup data untuk analisis

        # Ambil covariance matrix terbaru dan yang sebelumnya
        current_cov = self.angular_velocity_cov_history[-1]
        previous_cov = self.angular_velocity_cov_history[-2]

        # Analisis perubahan pada elemen covariance matrix terkait yaw (z)
        delta_cov_yaw = np.abs(current_cov[2, 2] - previous_cov[2, 2])

        # Tetapkan threshold untuk mendeteksi drift
        threshold = 0.01  # Sesuaikan nilai threshold ini sesuai kebutuhan

        if delta_cov_yaw > threshold:
            rospy.logwarn("IMU Yaw drift detected! Delta covariance: {}".format(delta_cov_yaw))
        else:
            rospy.loginfo("No significant IMU yaw drift. Delta covariance: {}".format(delta_cov_yaw))

    def detect_heading_drift(self):
        if len(self.yaw_history) < 2:
            return  # Tidak cukup data untuk analisis

        # Ambil nilai yaw terbaru dan yang sebelumnya
        current_yaw = self.yaw_history[-1]
        previous_yaw = self.yaw_history[-2]

        # Analisis perubahan pada nilai yaw
        delta_yaw = np.abs(current_yaw - previous_yaw)

        # Tetapkan threshold untuk mendeteksi drift
        threshold = 1.0  # Sesuaikan nilai threshold ini sesuai kebutuhan

        if delta_yaw > threshold:
            rospy.logwarn("Compass Yaw drift detected! Delta yaw: {:.2f}".format(delta_yaw))
        else:
            rospy.loginfo("No significant compass yaw drift. Delta yaw: {:.2f}".format(delta_yaw))

    def print_imu_data(self):
        # Konversi quaternion ke Euler angles
        euler = self.quaternion_to_euler(self.orientation)
        
        # Konversi radian ke derajat
        euler_deg = np.degrees(euler)
        euler_deg[2] += self.yaw_offset  # Tambahkan offset kalibrasi ke yaw

        angular_velocity_deg = np.degrees(self.angular_velocity)

        rospy.loginfo("IMU Orientation (degrees): Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}".format(euler_deg[0], euler_deg[1], euler_deg[2]))
        # rospy.loginfo("IMU Angular Velocity (degrees/sec): x: {:.2f}, y: {:.2f}, z: {:.2f}".format(angular_velocity_deg[0], angular_velocity_deg[1], angular_velocity_deg[2]))
        # rospy.loginfo("IMU Linear Acceleration (m/s^2): x: {:.2f}, y: {:.2f}, z: {:.2f}".format(self.linear_acceleration[0], self.linear_acceleration[1], self.linear_acceleration[2]))

    def print_heading_data(self):
        rospy.loginfo("Compass Yaw (degrees): {:.2f}".format(self.yaw))
        # rospy.loginfo("Compass Magnetometer Data: x: {:.2f}, y: {:.2f}, z: {:.2f}".format(self.mag_data[0], self.mag_data[1], self.mag_data[2]))

    def quaternion_to_euler(self, quaternion):
        # Mengonversi quaternion ke sudut Euler
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = SensorDriftDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
