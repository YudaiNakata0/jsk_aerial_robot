#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sensor_msgs.msg import Image

class LineDetection():
    def __init__(self, delta=10, cx=0, cy=0):
        self.bridge = CvBridge()
        self.image = Image()
        self.cv_image = []
        self.mono = []
        self.delta = delta
        self.cx = cx
        self.cy = cy
        self.polar_points = []
        self.count_list = []
        self.points_on_line = 0
        self.sub = rospy.Subscriber("/processed_image/mask", Image, self.callback)
        self.fig, self.ax = plt.subplots()
        self.ani = animation.FuncAnimation(self.fig, self.update, interval=100)
        
    def get_image(self, image):
        self.image = image
        #print(image)
        #array = np.frombuffer(image.data, dtype=np.uint8)
        #self.cv_image = cv2.imdecode(array, flags=cv2.IMREAD_COLOR)
        #print(self.cv_image)
        #cv2.imshow("Subscribed Image", self.cv_image)
        #cv2.waitKey(1)

    def get_points(self):
        self.mono = self.bridge.imgmsg_to_cv2(self.image, desired_encoding="mono8")
        ys, xs = np.where(self.mono == 255)
        self.points = list(zip(xs, ys))

    def cartesian_to_polar(self):
        self.polar_points = []
        num_division = int(180 // self.delta)
        self.count_list = [0]*num_division
        for point in self.points:
            x = point[0] - self.cx
            y = self.cy - point[1]
            if y > 0:
                r = np.sqrt(x**2 + y**2)
                theta = np.arctan2(y, x) * 180 / np.pi
                division = int(theta // self.delta)
                self.polar_points.append([r, theta, division])
                self.count_list[division] += 1

    def choose_line(self):
        max_index = np.argmax(self.count_list)
        self.points_on_line = self.count_list[max_index]
        arg = np.radians(self.delta*(max_index + 0.5))
        self.slope = np.tan(arg)

    def draw_line(self):
        image = cv2.cvtColor(self.mono, cv2.COLOR_GRAY2RGB)
        cv2.circle(image, (self.cx, self.cy), 5, (0, 255, 0), -1)
        if self.points_on_line > 30:
            if self.slope > 0:
                ex = 2*self.cx
                ey = self.cy - int(self.slope*self.cx)
                if ey < 0:
                    ey = 0
                    ex = self.cx + int(self.cy/self.slope)
            else:
                ex = 0
                ey = self.cy + int(self.slope*self.cx)
                if ey < 0:
                    ey = 0
                    ex = self.cx + int(self.cy/self.slope)
        
            cv2.line(image, (self.cx, self.cy), (ex, ey), (255, 0, 0), 4)
        if self.cv_image != None:
            cv2.imshow("Line", image)
            cv2.waitKey(1)
        
    def callback(self, msg):
        self.get_image(msg)
        self.get_points()
        self.cartesian_to_polar()
        self.choose_line()
        self.draw_line()

    def callback_graph(self, event):
        self.ax.cla()
        self.ax.hist(self.count_list, bins=30, alpha=0.7)
        self.ax.title("red points")
        plt.pause(0.1)

    def update(self, frame):
        self.ax.cla()
        self.ax.plot(range(len(self.count_list)), self.count_list, marker="o")
        #self.ax.hist(self.count_list, bins=30, alpha=0.7)
        self.ax.set_title("red points")
        self.ax.set_xlabel("argument")
        self.ax.set_ylabel("number of points")

    def draw_graph(self):
        plt.show()
        
if __name__ == "__main__":
    rospy.init_node("line_detection_node")
    Class = LineDetection(delta=5, cx=200, cy=300)
    #rospy.spin()
    Class.draw_graph()
