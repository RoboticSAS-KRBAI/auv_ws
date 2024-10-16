#!/usr/bin/env python

import rospy
import numpy as np
from robotic_sas_auv_ros.msg import Heading  # Ganti dengan pesan yang sesuai untuk data heading

class CompassDriftDetector:
    def __init__(self):
        rospy.init_node('compass_drift_detector', anonymous=True)
        self.subscriber = rospy.Subscriber('/witmotion/heading', Heading, self.callback_heading)
        
        # Inisialisasi data heading
        self.yaw = 0.0
        self.mag_data = np.zeros(3)
        
        # Buffer untuk menyimpan history heading
        self.yaw_history = []

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
        self.detect_drift()

        # Cetak data heading dalam derajat
        self.print_heading_data()

    def detect_drift(self):
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
            rospy.logwarn("Yaw drift detected! Delta yaw: {:.2f}".format(delta_yaw))
        else:
            rospy.loginfo("No significant yaw drift. Delta yaw: {:.2f}".format(delta_yaw))

    def print_heading_data(self):
        rospy.loginfo("Yaw (degrees): {:.2f}".format(self.yaw))
        rospy.loginfo("Magnetometer Data: x: {:.2f}, y: {:.2f}, z: {:.2f}".format(self.mag_data[0], self.mag_data[1], self.mag_data[2]))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = CompassDriftDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
