#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu
import tf

class IMUDriftDetector:
    def __init__(self):
        rospy.init_node('imu_drift_detector', anonymous=True)
        self.subscriber = rospy.Subscriber('/imu', Imu, self.callback_imu)
        
        # Inisialisasi data IMU
        self.orientation = np.zeros(4)
        self.angular_velocity = np.zeros(3)
        self.linear_acceleration = np.zeros(3)
        
        # Inisialisasi covariance matrices
        self.orientation_covariance = np.zeros((3, 3))
        self.angular_velocity_covariance = np.zeros((3, 3))
        self.linear_acceleration_covariance = np.zeros((3, 3))
        
        # Buffer untuk menyimpan history covariance matrices
        self.angular_velocity_cov_history = []

    def callback_imu(self, data: Imu):
        # Callback untuk data IMU dari topik /imu
        self.orientation = np.array([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
        self.angular_velocity = np.array([data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z])
        self.linear_acceleration = np.array([data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z])
        
        # Covariance matrices
        self.orientation_covariance = np.array(data.orientation_covariance).reshape((3, 3))
        self.angular_velocity_covariance = np.array(data.angular_velocity_covariance).reshape((3, 3))
        self.linear_acceleration_covariance = np.array(data.linear_acceleration_covariance).reshape((3, 3))

        # Simpan covariance matrix angular velocity dalam buffer history
        self.angular_velocity_cov_history.append(self.angular_velocity_covariance)
        
        # Jaga ukuran buffer agar tidak terlalu besar
        if len(self.angular_velocity_cov_history) > 100:
            self.angular_velocity_cov_history.pop(0)
        
        # Analisis drifting pada arah yaw (angular velocity z)
        self.detect_drift()

        # Cetak data IMU dalam derajat
        self.print_imu_data()

    def detect_drift(self):
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
            rospy.logwarn("Yaw drift detected! Delta covariance: {}".format(delta_cov_yaw))
        else:
            rospy.loginfo("No significant yaw drift. Delta covariance: {}".format(delta_cov_yaw))

    def print_imu_data(self):
        # Konversi quaternion ke Euler angles
        euler = self.quaternion_to_euler(self.orientation)
        
        # Konversi radian ke derajat
        euler_deg = np.degrees(euler)
        angular_velocity_deg = np.degrees(self.angular_velocity)

        rospy.loginfo("Orientation (degrees): Roll: {:.2f}, Pitch: {:.2f}, Yaw: {:.2f}".format(euler_deg[0], euler_deg[1], euler_deg[2]))
        # rospy.loginfo("Angular Velocity (degrees/sec): x: {:.2f}, y: {:.2f}, z: {:.2f}".format(angular_velocity_deg[0], angular_velocity_deg[1], angular_velocity_deg[2]))
        # rospy.loginfo("Linear Acceleration (m/s^2): x: {:.2f}, y: {:.2f}, z: {:.2f}".format(self.linear_acceleration[0], self.linear_acceleration[1], self.linear_acceleration[2]))

    def quaternion_to_euler(self, quaternion):
        # Mengonversi quaternion ke sudut Euler
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return euler

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = IMUDriftDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
