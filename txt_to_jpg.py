"""
USAGE: To turn the 2D array into .jpg image file
"""

import numpy as np 
import cv2

# odom_map = np.loadtxt('odometry_path.txt', delimiter=" ")
odom_map = np.genfromtxt('odometry_path.txt', dtype=float)

odom_map = 255*odom_map

cv2.imshow('My Image', odom_map)
cv2.imwrite('odom.jpg',odom_map)

cv2.waitKey(0)
cv2.destroyAllWindows()