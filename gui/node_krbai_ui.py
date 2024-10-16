#!/usr/bin/env python3
import sys
import rospy
from PyQt5 import QtWidgets, QtCore, QtGui
from krbai_ui import Ui_Dialog  # Make sure this is correctly imported from your project files
from robotic_sas_auv_ros.msg import ArduinoSensor, Sensor
from std_msgs.msg import Bool, Float32
import os

# Global variables
ros_process = None
status = False

class RobotInterface(QtWidgets.QDialog):
    def __init__(self):
        super(RobotInterface, self).__init__()
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.customSetupCode()

        # ROS Publishers and Subscribers
        rospy.Subscriber("/rosserial/sensor", ArduinoSensor, self.callback_voltage, queue_size=10)
        rospy.Subscriber('/nuc/sensor', Sensor, self.callback_nuc)
        rospy.Subscriber('/filterYaw', Float32, self.callback_yaw)
        self.status_publish = rospy.Publisher('robot_status', Bool, queue_size=10)

    def customSetupCode(self):
        self.ui.Logo_telkom.setPixmap(QtGui.QPixmap("/home/techsas/auv_ws/src/robotic_sas_auv_ros/gui/TelkomUniversity.png").scaled(431, 421, QtCore.Qt.KeepAspectRatio))
        self.ui.Logo_robotic.setPixmap(QtGui.QPixmap("/home/techsas/auv_ws/src/robotic_sas_auv_ros/gui/logo_robotic.png").scaled(431, 421, QtCore.Qt.KeepAspectRatio))
        self.set_opacity(self.ui.Logo_telkom, 0.5)
        self.set_opacity(self.ui.Logo_robotic, 0.5)
        self.ui.pushButton_Start.setStyleSheet('background-color: green; color : white')
        self.ui.pushButton_Stop.setStyleSheet('background-color: red; color : white')
        self.ui.Refresh_button.setStyleSheet('background-color: lightblue; color : white')
        self.ui.pushButton_Start.clicked.connect(self.runRobot)
        self.ui.pushButton_Stop.clicked.connect(self.stopRobot)
        self.ui.Refresh_button.clicked.connect(self.refresh)

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.timer.start(60000)  # Refresh every 10000 milliseconds (10 seconds)

    def refresh(self):
        os.system('clear')
        print("Refreshing data...")
        # Attempt to fetch new data
        self.ui.Voltage_label.setText(str(0))
        self.ui.Power_label.setText(str(0))
        self.ui.Depth_label.setText(str(0))
        self.ui.InternalPresure_label.setText(str(0))
        self.ui.Temperature_label.setText(str(0))

        self.fetch_sensor_data()
        print("UI Refreshed!!!")

        # Switch to tab 2 and then back to tab 1 after a short delay
        self.ui.tabWidget.setCurrentIndex(1)
        QtCore.QTimer.singleShot(500, self.switch_back_to_tab1)  # 500 milliseconds delay
    
    def set_opacity(self, widget, opacity):
        effect = QtWidgets.QGraphicsOpacityEffect()
        effect.setOpacity(opacity)
        widget.setGraphicsEffect(effect)

    def switch_back_to_tab1(self):
        self.ui.tabWidget.setCurrentIndex(0)

    def fetch_sensor_data(self):
        try:
            arduino_sensor_data = rospy.wait_for_message("/rosserial/sensor", ArduinoSensor, timeout=1.0)
            self.callback_voltage(arduino_sensor_data)
            nuc_sensor_data = rospy.wait_for_message("/nuc/sensor", Sensor, timeout=1.0)
            self.callback_nuc(nuc_sensor_data)
            yaw_data = rospy.wait_for_message("/filterYaw", Float32, timeout=1.0)
            self.callback_yaw(yaw_data)
        except rospy.ROSException as e:
            print("Failed to fetch new data:", e)

    def runRobot(self):
        global status
        status = True
        print("Robot started:", status)
        self.status_publish.publish(status)

    def stopRobot(self):
        global status
        status = False
        print("Robot stopped:", status)
        self.status_publish.publish(status)

    def callback_voltage(self, data):
        global voltage
        voltage = data.loadvoltage
        self.ui.Current_label.setText(str(data.current_mA))
        self.ui.Voltage_label.setText(str(voltage))
        self.ui.Power_label.setText(str(data.power_mW / 1000))
        self.ui.Depth_label.setText(str(data.depth))
        self.ui.InternalPresure_label.setText(str(data.pressure_relative))
        self.ui.Temperature_label.setText(str(data.temperature))
        if voltage < 22.6 and (data.power_mW / 1000) < 105:
            self.ui.Voltage_label.setStyleSheet('background-color: red; color : white')
        elif (voltage >=22.6 and voltage < 22.9 ) and (data.power_mW / 1000) < 105 :
            self.ui.Voltage_label.setStyleSheet('background-color: yellow; color : black')
        else:
            self.ui.Voltage_label.setStyleSheet('background-color: lightgreen; color : black')

    def callback_nuc(self, data):
        self.ui.Roll_label.setText(str(data.roll))
        self.ui.Pitch_label.setText(str(data.pitch))

    def callback_yaw(self, data):
        yaw = data.data
        self.ui.Yaw_label.setText(str(yaw))

if __name__ == "__main__":
    rospy.init_node("krbai")
    app = QtWidgets.QApplication(sys.argv)
    dialog = RobotInterface()
    dialog.show()
    sys.exit(app.exec_())
