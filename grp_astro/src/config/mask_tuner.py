#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2
import time
sys.path.append("..")
from cam_vision import CameraVision

camera = CameraVision(isRos=False) # Init camera 
# VARIABLES
camera._low = np.array([179, 255, 255]) # Set to max so detected values will be inferior
camera._high = np.array([0, 0, 0]) #Set to min so detected values will be superior
camera._kernel = np.ones((5, 5), np.uint8)


# Skip the first frames of the camera (for some reason they don't work well)
for i in range(20):
    frames = camera._pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()


if color_frame is None:
    exit


# Convert image to numpy arrays
color_image = np.asanyarray(color_frame.get_data())

# Convert image to HSV for easier thresholding
hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)


# Define bottle area by mouse
r = cv2.selectROI('Select region', color_image)
selectedRegion = hsv_image[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
cv2.destroyWindow('Select region')

# Calculate new HSV mask
H = selectedRegion[:,:,0].flatten()
S = selectedRegion[:,:,1].flatten()
V = selectedRegion[:,:,2].flatten()
hMargin = 13
sMargin = 10
vMargin = 10

camera._low = np.array([min(H) - hMargin, min(S) - sMargin, min(V) - vMargin])
camera._high = np.array([max(H) + hMargin, max(S) + sMargin, max(V) + vMargin])
print("-> Low and high HSV saved in 'hsv_params' file :",camera._low, camera._high)

# Vision
mask = camera.maskImage(hsv_image)

mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

# Show images
images = np.hstack((color_image, mask_rgb))

# Show images
cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RealSense', images)
cv2.waitKey(0)

# Save values in a file
file = open("hsv_params", "w")
file.write(str(camera._low) + "," + str(camera._high))
file.close()
# Stop streaming
print("\nEnding...")
camera._pipeline.stop()