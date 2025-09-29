#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sensor_msgs.msg import Image

class LineDetection():
    def __init__(self, delta=10, delta_r=10, cx=0, cy=0):
        self.bridge = CvBridge()
        self.image = Image()
        self.cv_image = []
        self.mono = []
        self.delta = delta
        self.delta_r = delta_r 
        self.cx = cx
        self.cy = cy
        self.polar_points = []
        self.count_list = []
        self.distribution_r = []
        self.max_index = 0
        self.max_r = 0
        self.points_on_line = 0
        self.sub = rospy.Subscriber("/processed_image/mask", Image, self.callback)
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2)
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
        self.max_index = np.argmax(self.count_list)
        self.points_on_line = self.count_list[self.max_index]
        arg = np.radians(self.delta*(self.max_index + 0.5))
        self.slope = np.tan(arg)

    def distribute_radius(self):
        num_division = int(500 // self.delta_r)
        self.distribution_r = [0]*num_division
        for point in self.polar_points:
            if point[2] == self.max_index:
                division_r = int(point[0] // self.delta_r)
                self.distribution_r[division_r] += 1

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
        self.distribute_radius()
        self.draw_line()

    def update(self, frame):
        self.ax1.cla()
        self.ax1.plot(range(len(self.count_list)), self.count_list, marker="o")
        #self.ax.hist(self.count_list, bins=30, alpha=0.7)
        self.ax1.set_title("red points(angle)")
        self.ax1.set_xlabel("argument")
        self.ax1.set_ylabel("number of points")
        xticks_1 = range(0, len(self.count_list)+1, 2)
        xtick_labels_1 = [x * self.delta for x in xticks_1]
        self.ax1.set_xticks(xticks_1)
        self.ax1.set_xticklabels(xtick_labels_1)
        self.ax1.set_ylim(-50, 1000)

        self.ax2.cla()
        self.ax2.plot(range(len(self.distribution_r)), self.distribution_r, marker="o")
        self.ax2.set_title("red points on the line(radius)")
        self.ax2.set_xlabel("radius")
        self.ax2.set_ylabel("number of points")
        xticks_2 = range(0, len(self.distribution_r)+1, 5)
        xtick_labels_2 = [x * self.delta_r for x in xticks_2]
        self.ax2.set_xticks(xticks_2)
        self.ax2.set_xticklabels(xtick_labels_2)
        #self.ax2.set_xlim(-5, 55)
        self.ax2.set_ylim(-10, 300)

    def draw_graph(self):
        plt.show()

    def cleanup(self):
        plt.close("all")
        
if __name__ == "__main__":
    rospy.init_node("line_detection_node")
    Class = LineDetection(delta=5, delta_r=20, cx=200, cy=300)
    rospy.on_shutdown(Class.cleanup)
    #Class.draw_graph()
    plt.show()
    #rospy.spin()
