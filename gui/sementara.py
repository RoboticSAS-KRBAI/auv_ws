#!/usr/bin/env python3
import os
import signal
import subprocess
from PyQt5 import QtCore, QtGui, QtWidgets
from krbai_ui import *
from robotic_sas_auv_ros.msg import ArduinoSensor, Sensor
import rospy
import time

# Variabel global untuk menyimpan proses yang dijalankan
ros_process = None
voltage = 0.
status = False

# Global variables to store sensor data
# sensor_data = {
#     "loadvoltage": 0.0,
#     "temperature": 0.0,
#     "pressure_relative": 0.0,
#     "current_mA": 0.0,
#     "depth": 0.0,
#     "power_mW": 0.0,
#     "roll": 0.0,
#     "pitch": 0.0,
#     "yaw": 0.0,
# }

def customSetupCode():
    global voltage
    ui.pushButton_Start.setStyleSheet('background-color: green; color : white')
    ui.pushButton_Stop.setStyleSheet('background-color: red; color : white')
    ui.Refresh_button.setStyleSheet('background-color: lightblue; color : white')
    ui.pushButton_Start.clicked.connect(runRobot)
    ui.pushButton_Stop.clicked.connect(stopRobot)
    ui.Refresh_button.clicked.connect(refresh)
    # print(voltage)

def refresh():
    

    # Force the UI elements to update
    # ui.Sensor_label.update()
    # ui.Voltage_label.update()
    # ui.Temperature_label.update()
    # ui.InternalPresure_label.update()
    # ui.Current_label.update()
    # ui.Depth_label.update()
    # ui.Power_label.update()
    # ui.Roll_label.update()
    # ui.Pitch_label.update()
    # ui.Yaw_label.update()

    print("Refreshed!!!")

def runRobot():
    global ros_process, status
    status = True
    print(status)

def stopRobot():
    global ros_process, status
    status = False

def callback_voltage(data: ArduinoSensor):
    global voltage
    voltage = data.loadvoltage

    # ui.Current_label.setText(str(data.current_mA))
    # ui.Voltage_label.setText(str(voltage))
    # ui.Power_label.setText(str(data.power_mW))
    # ui.Depth_label.setText(str(data.depth))
    # ui.InternalPresure_label.setText(str(data.pressure_relative))
    # ui.Temperature_label.setText(str(data.temperature))
    print(voltage)

    if voltage < 22.6:
        ui.Voltage_label.setStyleSheet('background-color: red; color : white')
    else:
        ui.Voltage_label.setStyleSheet('background-color: lightgreen; color : black')


def callback_nuc(data: Sensor):
    # sensor_data["roll"] = data.roll
    # sensor_data["pitch"] = data.pitch
    # sensor_data["yaw"] = data.yaw
    pass

if __name__ == "__main__":
    import sys
    rospy.init_node("krbai")
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    customSetupCode()

    # wheelSubscriber = rospy.Subscriber("Wheel",Wheel,lambda msg:ui.label.setText("Wheel Monitor\n\n"+str(1000)),queue_size=10)
    rospy.Subscriber("/rosserial/sensor", ArduinoSensor, lambda msg: ui.Sensor_label.setText(str(msg)), queue_size=10)
    rospy.Subscriber("/rosserial/sensor", ArduinoSensor, callback_voltage, queue_size=10)

    rospy.Subscriber("/nuc/sensor", Sensor, callback_nuc, queue_size=10)
    rospy.Subscriber("/nuc/sensor", Sensor, lambda msg: ui.Roll_label.setText(str(msg.roll)), queue_size=10)
    rospy.Subscriber("/nuc/sensor", Sensor, lambda msg: ui.Pitch_label.setText(str(msg.pitch)), queue_size=10)
    rospy.Subscriber("/nuc/sensor", Sensor, lambda msg: ui.Yaw_label.setText(str(msg.yaw)), queue_size=10)

    Dialog.show()
    sys.exit(app.exec_())
