#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String, Int8
from geometry_msgs.msg import Point
import threading
import time

class Map:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.axhline(y=0, color='k')  # horizontal line
        self.ax.axvline(x=0, color='k')  # vertical line
        self.ax.set_xlim(-2.5, 2.5)
        self.ax.set_ylim(0, 30)
        self.ax.set_yticks(np.arange(0, 31, 1))
        self.ax.grid(True)
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')

        # Add rectangles
        gate_1 = plt.Rectangle((-0.9, 8), 1.8, 0.5, color='orange')
        gate_2 = plt.Rectangle((-1.9, 19), 1.8, 0.5, color='orange')
        gate_3 = plt.Rectangle((0.1, 19), 1.8, 0.5, color='orange')
        tarpaulin = plt.Rectangle((-0.6, 24), 1.2, 1.2, color='green')
        self.ax.add_patch(gate_1)
        self.ax.add_patch(gate_2)
        self.ax.add_patch(gate_3)
        self.ax.add_patch(tarpaulin)

        # Add circles
        obstacle = plt.Circle((0, 11), 0.2, color='orange')
        bucket = plt.Circle((0, 24.5), 0.35, color='red')
        self.ax.add_patch(obstacle)
        self.ax.add_patch(bucket)

        # Create the rectangle for animation
        self.rect = plt.Rectangle((-0.1, 0), 0.2, 0.2, color='blue')
        self.ax.add_patch(self.rect)

        # Define the speed (per frame)
        self.speed = 0.5 #0.42 #0.6 brutal
        # self.speed_camera = 0.28  #(ga pakai gate)  
        # self.speed_camera = 0.3
        self.speed_camera = 0.27
        self.turn_speed = 0.15

        # Show the plot
        plt.ion()
        plt.show()

    def animate(self, move):
        print("move:", move)
        if move == "stop":
            pass
        if move == "forward" or move == "last":  # Move forward
            self.rect.set_y(self.rect.get_y() + self.speed)
        elif move == "right":  # Turn right 
            self.rect.set_x(self.rect.get_x() + self.turn_speed)
        elif move == "left":  # Turn left
            self.rect.set_x(self.rect.get_x() - self.turn_speed)
        elif move == "camera":
            self.rect.set_y(self.rect.get_y() + self.speed_camera)

        # Print the coordinates of the rectangle
        print(f"(x, y) = ({self.rect.get_x() + 0.10:.2f}, {self.rect.get_y():.2f})")

        # Redraw the figure
        self.fig.canvas.draw_idle()

class Subscriber:
    def __init__(self):
        self.plot = Map()
        self.last_time = time.time()
        self.position = Point()

        # Subscriber
        rospy.Subscriber('/nuc/move', String, self.callback_move)

        # Publisher
        self.pub_flag = rospy.Publisher('/nuc/flag', Int8, queue_size=10)
        self.pub_position = rospy.Publisher('position', Point, queue_size=10)
        self.last_flag_published = 0

        rospy.loginfo("Node Map Has Been Started")

    def callback_move(self, data: String):
        self.move = data.data

    def check_and_publish_flag(self):
        current_y = self.plot.rect.get_y()
        if current_y >= 24 and self.last_flag_published <= 3:
            self.pub_flag.publish(3)
            self.last_flag_published = 3
            rospy.loginfo("Flag right now is 3")
        elif current_y > 12 and self.last_flag_published <= 2:
            self.pub_flag.publish(2)
            self.last_flag_published = 2
            rospy.loginfo("Flag right now is 2")
        elif current_y > 8 and self.last_flag_published <= 1:
            self.pub_flag.publish(1)
            self.last_flag_published = 1
            rospy.loginfo("Flag right now is 1")
        else:
            self.pub_flag.publish(0)
            rospy.loginfo("Flag right now is 0")
        
    def spin(self):
        # Run ROS spin in a separate thread
        spin_thread = threading.Thread(target=rospy.spin)
        spin_thread.start()

        # Keep matplotlib plot responsive
        while not rospy.is_shutdown():
            current_time = time.time()
            if current_time - self.last_time >= 1:  # Check if 1 second has passed
                if hasattr(self, 'move'):
                    self.plot.animate(self.move)
                    self.position.x = self.plot.rect.get_x()
                    self.position.y = self.plot.rect.get_y()
                    self.pub_position.publish(self.position)
                    self.check_and_publish_flag()
                    self.last_time = current_time
            plt.pause(0.1)

def main():
    rospy.init_node('node_map', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()
