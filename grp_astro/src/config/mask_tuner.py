#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2
import time

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

print( f"Connect: {device_product_line}" )
found_rgb = True
for s in device.sensors:
    print( "Name:" + s.get_info(rs.camera_info.name) )
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True

if not (found_rgb):
    print("Depth camera required !!!")
    exit(0)

config.enable_stream(rs.stream.color, width=848, height=480, format=rs.format.bgr8, framerate=60)
config.enable_stream(rs.stream.depth, width=848, height=480, format=rs.format.z16, framerate=60)

# Capture ctrl-c event
isOk= True
def signalInterruption(signum, frame):
    global isOk
    print( "\nCtrl-c pressed" )
    isOk= False

signal.signal(signal.SIGINT, signalInterruption)

# Start streaming
pipeline.start(config)

# VARIABLES
low = np.array([179, 255, 255]) # Set to max so detected values will be inferior
high = np.array([0, 0, 0]) #Set to min so detected values will be superior
kernel = np.ones((5, 5), np.uint8)


# Skip the first frames of the camera (for some reason they don't work well)
for i in range(20):
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()


if color_frame is not None:
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

low = np.array([min(H) - hMargin, min(S) - sMargin, min(V) - vMargin])
high = np.array([max(H) + hMargin, max(S) + sMargin, max(V) + vMargin])
print("-> Low and high HSV saved in 'hsv_params' file :",low, high)

# Vision
# Seuillage par colorimétrie
mask = cv2.inRange(hsv_image, low, high)

# Réduction du bruit
mask = cv2.erode(mask, kernel, iterations=4)
mask = cv2.dilate(mask, kernel, iterations=4)

mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

# Show images
images = np.hstack((color_image, mask_rgb))

# Show images
cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RealSense', images)
cv2.waitKey(0)

# Save values in a file
file = open("hsv_params", "w")
file.write(str(low) + "," + str(high))
file.close()
# Stop streaming
print("\nEnding...")
pipeline.stop()