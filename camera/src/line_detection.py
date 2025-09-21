#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image

def cartesian_to_polar(points, cx, cy, delta):
    polar_points = []
    num_region = int(180 // delta)
    count_list = [0]*num_region
    for point in points:
        x = point[0] - cx
        y = point[1] - cy
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y, x) * 180 / np.pi
        region = int(theta // delta)
        polar_points.append([r, theta, region])
        count_list[region] += 1
        #print(count_list)
    return polar_points, count_list

def choose_line(polar_points, count_list):
    max_index = np.argmax(count_list)
    degree = max_index * 10 + 5
    slope = np.tan(np.radians(degree))
    return slope

def points_generate(N, x_range, y_range, seed):
    rng = np.random.default_rng(seed)
    xs = rng.uniform(x_range[0], x_range[1], size=N)
    ys = rng.uniform(y_range[0], y_range[1], size=N)
    points = [(float(x), float(y)) for x, y in zip(xs, ys)]
    return xs, ys, points

def show_graph(xs, ys, slope):
    plt.figure(figsize=(8, 6))
    plt.scatter(xs, ys, alpha=0.7)

    if slope > 0:
        x_line = np.linspace(0, x_range[1], 200)
    else:
        x_line = np.linspace(x_range[0], 0, 200)
    y_line = slope * x_line
    plt.plot(x_line, y_line, color="red", linewidth=2, label="")

    plt.title(f"Scatter of {N} Random Points")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.grid(True)
    plt.show()

def callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
    print(cv_image)
    ys, xs = np.where(cv_image == 255)
    points = list(zip(xs, ys))
    polar_points, count_list = cartesian_to_polar(points, 200, 100, 10)
    slope = choose_line(polar_points, count_list)
    
    
if __name__ == "__main__":
    rospy.init_node("line_detection_node")
    rospy.Subscriber("/processed_image/mask", Image, callback)
    rospy.spin()
